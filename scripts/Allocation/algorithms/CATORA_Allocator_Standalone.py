#!/usr/bin/env python3
"""
CAT-ORA Standalone Allocator
Collision-Aware Time-Optimal formation Reshaping Algorithm
Pure Python implementation - no ROS2 required
"""

import numpy as np
from scipy.optimize import linear_sum_assignment
from typing import List, Tuple
import time


class CATORAStandaloneAllocator:
    """
    CAT-ORA 알고리즘의 Python standalone 구현
    - Hungarian 알고리즘으로 초기 할당
    - 충돌 감지 (궤적 기반)
    - Branch & Bound로 충돌 회피 할당 탐색
    - Min-max 최적화 (bottleneck 최소화)
    """

    def __init__(self, max_velocity=2.0, max_acceleration=2.0, min_distance=1.0):
        """
        Args:
            max_velocity: 최대 속도 (m/s)
            max_acceleration: 최대 가속도 (m/s²)
            min_distance: 최소 안전 거리 (m)
        """
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.min_distance = min_distance

    def calculate_distance(self, pos1, pos2):
        """두 점 사이의 유클리드 거리 계산"""
        return np.linalg.norm(np.array(pos1) - np.array(pos2))

    def compute_distance_matrix(self, starts: List, goals: List) -> np.ndarray:
        """
        거리 행렬 계산

        Args:
            starts: 시작 위치 리스트 [[x,y,z], ...]
            goals: 목표 위치 리스트 [[x,y,z], ...]

        Returns:
            distance_matrix[i][j]: 드론 i에서 목표 j까지의 거리
        """
        n = len(starts)
        distance_matrix = np.zeros((n, n))

        for i in range(n):
            for j in range(n):
                distance_matrix[i][j] = self.calculate_distance(starts[i], goals[j])

        return distance_matrix

    def hungarian_assignment(self, distance_matrix: np.ndarray) -> Tuple[List[int], float]:
        """
        Hungarian 알고리즘으로 초기 할당

        Args:
            distance_matrix: 거리 행렬

        Returns:
            assignment: 드론 ID → 목표 ID 매핑
            max_distance: 최대 거리 (bottleneck)
        """
        row_ind, col_ind = linear_sum_assignment(distance_matrix)
        assignment = col_ind.tolist()

        # 최대 거리 계산 (bottleneck)
        max_distance = max(distance_matrix[i, assignment[i]] for i in range(len(assignment)))

        return assignment, max_distance

    def compute_trajectory_time(self, distance: float) -> float:
        """
        거리에 대한 최소 비행 시간 계산 (bang-bang control)

        Args:
            distance: 비행 거리

        Returns:
            time: 최소 비행 시간
        """
        # Bang-bang control: 가속 → 등속 → 감속
        # t_acc = v_max / a_max
        # d_acc = 0.5 * a_max * t_acc^2

        t_acc = self.max_velocity / self.max_acceleration
        d_acc = 0.5 * self.max_acceleration * t_acc ** 2

        # 전체 거리가 가속+감속 거리보다 짧으면
        if distance <= 2 * d_acc:
            # 가속만 하다가 바로 감속
            return 2 * np.sqrt(distance / self.max_acceleration)
        else:
            # 가속 → 등속 → 감속
            d_const = distance - 2 * d_acc
            t_const = d_const / self.max_velocity
            return 2 * t_acc + t_const

    def check_trajectory_collision(self, start1: np.ndarray, goal1: np.ndarray,
                                   start2: np.ndarray, goal2: np.ndarray) -> bool:
        """
        두 직선 궤적 사이의 최소 거리가 안전 거리보다 작은지 확인

        Args:
            start1, goal1: 드론 1의 시작/목표 위치
            start2, goal2: 드론 2의 시작/목표 위치

        Returns:
            True if collision detected (min distance < min_distance)
        """
        # 두 직선 세그먼트 사이의 최소 거리 계산
        # P1(t) = start1 + t * (goal1 - start1), t ∈ [0, 1]
        # P2(s) = start2 + s * (goal2 - start2), s ∈ [0, 1]

        d1 = goal1 - start1  # 방향 벡터 1
        d2 = goal2 - start2  # 방향 벡터 2
        w0 = start1 - start2

        a = np.dot(d1, d1)  # ||d1||^2
        b = np.dot(d1, d2)
        c = np.dot(d2, d2)  # ||d2||^2
        d = np.dot(d1, w0)
        e = np.dot(d2, w0)

        denom = a * c - b * b

        # 평행한 경우
        if abs(denom) < 1e-6:
            # 시작점과 끝점 사이의 거리 확인
            distances = [
                np.linalg.norm(start1 - start2),
                np.linalg.norm(start1 - goal2),
                np.linalg.norm(goal1 - start2),
                np.linalg.norm(goal1 - goal2)
            ]
            return min(distances) < self.min_distance

        # 최소 거리가 되는 매개변수 계산
        t = (b * e - c * d) / denom
        s = (a * e - b * d) / denom

        # [0, 1] 범위로 클램핑
        t = np.clip(t, 0, 1)
        s = np.clip(s, 0, 1)

        # 최소 거리 계산
        p1 = start1 + t * d1
        p2 = start2 + s * d2
        min_dist = np.linalg.norm(p1 - p2)

        return min_dist < self.min_distance

    def detect_collisions(self, starts: List, goals: List, assignment: List[int]) -> bool:
        """
        주어진 할당에서 충돌이 발생하는지 확인

        Args:
            starts: 시작 위치 리스트
            goals: 목표 위치 리스트
            assignment: 할당 (드론 i → 목표 assignment[i])

        Returns:
            True if any collision detected
        """
        n = len(starts)

        for i in range(n):
            for j in range(i + 1, n):
                start_i = np.array(starts[i])
                goal_i = np.array(goals[assignment[i]])
                start_j = np.array(starts[j])
                goal_j = np.array(goals[assignment[j]])

                if self.check_trajectory_collision(start_i, goal_i, start_j, goal_j):
                    return True

        return False

    def bottleneck_hungarian(self, distance_matrix: np.ndarray) -> Tuple[List[int], float]:
        """
        Bottleneck Assignment Problem을 Hungarian으로 근사 해결
        Min-max 최적화 (최대 거리 최소화)

        Args:
            distance_matrix: 거리 행렬

        Returns:
            assignment: 최적 할당
            bottleneck: 최대 거리
        """
        n = len(distance_matrix)

        # 거리 행렬의 고유값들을 threshold 후보로 사용
        unique_distances = np.unique(distance_matrix)
        unique_distances = np.sort(unique_distances)

        best_assignment = None
        best_bottleneck = float('inf')

        # Binary search로 최소 bottleneck 찾기
        for threshold in unique_distances:
            # threshold보다 작거나 같은 거리만 사용
            bounded_matrix = np.where(distance_matrix <= threshold, distance_matrix, 1e9)

            # Hungarian 알고리즘 적용
            row_ind, col_ind = linear_sum_assignment(bounded_matrix)
            assignment = col_ind.tolist()

            # 실제 최대 거리 계산
            max_dist = max(distance_matrix[i, assignment[i]] for i in range(n))

            if max_dist < best_bottleneck:
                best_bottleneck = max_dist
                best_assignment = assignment

        return best_assignment, best_bottleneck

    def swap_assignment(self, assignment: List[int], i: int, j: int) -> List[int]:
        """할당에서 두 목표를 교환"""
        new_assignment = assignment.copy()
        new_assignment[i], new_assignment[j] = new_assignment[j], new_assignment[i]
        return new_assignment

    def compute_bottleneck(self, distance_matrix: np.ndarray, assignment: List[int]) -> float:
        """할당의 bottleneck (최대 거리) 계산"""
        return max(distance_matrix[i, assignment[i]] for i in range(len(assignment)))

    def branch_and_bound(self, starts: List, goals: List,
                        initial_assignment: List[int],
                        max_iterations: int = 100) -> Tuple[List[int], float]:
        """
        Branch & Bound로 충돌 없는 할당 탐색

        Args:
            starts: 시작 위치 리스트
            goals: 목표 위치 리스트
            initial_assignment: 초기 할당 (Hungarian)
            max_iterations: 최대 반복 횟수

        Returns:
            assignment: 충돌 없는 최적 할당
            bottleneck: 최대 거리
        """
        n = len(starts)
        distance_matrix = self.compute_distance_matrix(starts, goals)

        # 초기 할당 확인
        if not self.detect_collisions(starts, goals, initial_assignment):
            bottleneck = self.compute_bottleneck(distance_matrix, initial_assignment)
            return initial_assignment, bottleneck

        # Branch & Bound 탐색
        best_assignment = initial_assignment
        best_bottleneck = self.compute_bottleneck(distance_matrix, initial_assignment)

        # Local search: 이웃 할당 탐색
        for iteration in range(max_iterations):
            improved = False

            # 모든 가능한 swap 시도
            for i in range(n):
                for j in range(i + 1, n):
                    new_assignment = self.swap_assignment(best_assignment, i, j)

                    # 충돌 확인
                    if self.detect_collisions(starts, goals, new_assignment):
                        continue

                    # Bottleneck 계산
                    new_bottleneck = self.compute_bottleneck(distance_matrix, new_assignment)

                    # 더 나은 할당이면 업데이트
                    if new_bottleneck < best_bottleneck:
                        best_assignment = new_assignment
                        best_bottleneck = new_bottleneck
                        improved = True
                        break

                if improved:
                    break

            # 개선이 없으면 종료
            if not improved:
                break

        return best_assignment, best_bottleneck

    def allocate(self, drones: List, goals: List, verbose: bool = True) -> List[int]:
        """
        CAT-ORA 알고리즘으로 드론을 목표에 할당

        Args:
            drones: 드론 위치 리스트 [[x,y,z], ...]
            goals: 목표 위치 리스트 [[x,y,z], ...]
            verbose: 상세 출력 여부

        Returns:
            assignment: 드론 인덱스 → 목표 인덱스 매핑 리스트
        """
        start_time = time.time()

        if verbose:
            print(f'Computing CAT-ORA assignment for {len(drones)} drones...')

        # 1. 거리 행렬 계산
        distance_matrix = self.compute_distance_matrix(drones, goals)

        # 2. Bottleneck Hungarian으로 초기 할당
        initial_assignment, initial_bottleneck = self.bottleneck_hungarian(distance_matrix)

        if verbose:
            print(f'Initial Hungarian assignment: bottleneck = {initial_bottleneck:.2f}m')

        # 3. 충돌 감지
        has_collision = self.detect_collisions(drones, goals, initial_assignment)

        if has_collision:
            if verbose:
                print('Collision detected! Running Branch & Bound...')

            # 4. Branch & Bound로 충돌 회피 할당 탐색
            final_assignment, final_bottleneck = self.branch_and_bound(
                drones, goals, initial_assignment, max_iterations=100
            )

            if verbose:
                print(f'Final assignment: bottleneck = {final_bottleneck:.2f}m')
        else:
            if verbose:
                print('No collision detected!')
            final_assignment = initial_assignment
            final_bottleneck = initial_bottleneck

        elapsed_time = (time.time() - start_time) * 1000  # ms

        if verbose:
            print(f'CAT-ORA assignment computed in {elapsed_time:.1f}ms')
            self._print_allocation_result(drones, goals, final_assignment)

        return final_assignment

    def _print_allocation_result(self, drones, goals, assignment):
        """할당 결과 출력"""
        print("\n=== CAT-ORA Allocation Result ===")
        distances = []

        for drone_id, goal_id in enumerate(assignment):
            distance = self.calculate_distance(drones[drone_id], goals[goal_id])
            distances.append(distance)
            print(f"Drone {drone_id} → Goal {goal_id}: {distance:.2f}m")

        print(f"\nTotal distance: {sum(distances):.2f}m")
        print(f"Average distance: {np.mean(distances):.2f}m")
        print(f"Max distance (bottleneck): {max(distances):.2f}m")
        print(f"Min distance: {min(distances):.2f}m")
        print(f"Std deviation: {np.std(distances):.2f}m")
        print("=" * 35)


def test_allocator():
    """Test the CATORA standalone allocator with sample data"""
    allocator = CATORAStandaloneAllocator(
        max_velocity=2.0,
        max_acceleration=2.0,
        min_distance=1.0
    )

    # Test data - 3 drones
    drones = [
        [0.0, 0.0, 3.0],
        [0.0, 1.0, 3.0],
        [0.0, 2.0, 3.0],
    ]

    goals = [
        [10.0, 2.0, 3.0],
        [10.0, 1.0, 3.0],
        [10.0, 0.0, 3.0],
    ]

    print("=== CAT-ORA Standalone Allocator Test ===")
    print(f"Drones: {drones}")
    print(f"Goals: {goals}")
    print()

    assignment = allocator.allocate(drones, goals)

    print(f"\nFinal assignment: {assignment}")


if __name__ == '__main__':
    test_allocator()
