# hungarian_allocator.py
# Fair Min-Max Hungarian Allocator
# 목표: 최대 거리를 최소화 (배터리 소모 균등화)

import numpy as np
from scipy.optimize import linear_sum_assignment


class HungarianAllocator:
    """
    공평한 헝가리안 할당 알고리즘
    - 기본 헝가리안으로 초기 할당
    - 최대 거리를 최소화하도록 반복 최적화
    """

    def __init__(self, max_iterations=50):
        """
        Args:
            max_iterations: 최적화 반복 횟수
        """
        self.max_iterations = max_iterations

    def calculate_distance(self, pos1, pos2):
        """두 점 사이의 유클리드 거리 계산"""
        return np.linalg.norm(np.array(pos1) - np.array(pos2))

    def create_cost_matrix(self, drones, goals):
        """
        비용 행렬 생성

        Args:
            drones: 드론 위치 리스트 [[x,y,z], ...]
            goals: 목표 위치 리스트 [[x,y,z], ...]

        Returns:
            cost_matrix: NxN 거리 행렬
        """
        n = len(drones)
        cost_matrix = np.zeros((n, n))

        for i in range(n):
            for j in range(n):
                cost_matrix[i][j] = self.calculate_distance(drones[i], goals[j])

        return cost_matrix

    def allocate(self, drones, goals):
        """
        드론을 목표에 할당

        Args:
            drones: 드론 위치 리스트 [[x,y,z], ...]
            goals: 목표 위치 리스트 [[x,y,z], ...]

        Returns:
            assignment: 드론 인덱스 → 목표 인덱스 매핑 리스트
                       assignment[i] = j 는 드론 i가 목표 j로 할당됨을 의미
        """
        # 1. 비용 행렬 생성
        cost_matrix = self.create_cost_matrix(drones, goals)

        # 2. 표준 헝가리안 알고리즘 (총 거리 최소화)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # 3. 결과 변환 (row_ind는 [0,1,2,...,n-1]이므로 col_ind만 필요)
        assignment = col_ind.tolist()

        # 4. 할당 결과 출력
        self._print_allocation_result(drones, goals, assignment, cost_matrix)

        return assignment

    def allocate_fair(self, drones, goals):
        """
        공평한 할당 (최대 거리 최소화)

        Args:
            drones: 드론 위치 리스트
            goals: 목표 위치 리스트

        Returns:
            assignment: 최적 할당
        """
        cost_matrix = self.create_cost_matrix(drones, goals)
        n = len(drones)

        # 초기 할당 (표준 헝가리안)
        _, best_assignment = linear_sum_assignment(cost_matrix)
        best_max_distance = self._get_max_distance(cost_matrix, best_assignment)

        # 반복 최적화: 최대 거리를 줄이는 방향으로 개선
        for iteration in range(self.max_iterations):
            improved = False

            # 모든 swap 조합 시도
            for i in range(n):
                for j in range(i+1, n):
                    # i와 j의 목표를 바꿔봄
                    new_assignment = best_assignment.copy()
                    new_assignment[i], new_assignment[j] = new_assignment[j], new_assignment[i]

                    new_max_distance = self._get_max_distance(cost_matrix, new_assignment)

                    # 최대 거리가 줄어들면 채택
                    if new_max_distance < best_max_distance:
                        best_assignment = new_assignment
                        best_max_distance = new_max_distance
                        improved = True

            # 개선이 없으면 종료
            if not improved:
                print(f"Fair allocation converged at iteration {iteration}")
                break

        # 결과 출력
        print("\n=== Fair Allocation Result ===")
        self._print_allocation_result(drones, goals, best_assignment, cost_matrix)

        return best_assignment.tolist()

    def _get_max_distance(self, cost_matrix, assignment):
        """할당에서 최대 거리 계산"""
        distances = [cost_matrix[i][assignment[i]] for i in range(len(assignment))]
        return max(distances)

    def _print_allocation_result(self, drones, goals, assignment, cost_matrix):
        """할당 결과 출력"""
        print("\n=== Allocation Result ===")
        distances = []

        for drone_id, goal_id in enumerate(assignment):
            distance = cost_matrix[drone_id][goal_id]
            distances.append(distance)
            print(f"Drone {drone_id} → Goal {goal_id}: {distance:.2f}m")

        print(f"\nTotal distance: {sum(distances):.2f}m")
        print(f"Average distance: {np.mean(distances):.2f}m")
        print(f"Max distance: {max(distances):.2f}m")
        print(f"Min distance: {min(distances):.2f}m")
        print(f"Std deviation: {np.std(distances):.2f}m")
        print("=" * 30)


# 테스트 코드
if __name__ == "__main__":
    # 간단한 테스트
    allocator = HungarianAllocator()

    # 테스트 데이터
    drones = [
        [0, 0, 3],
        [0, 1, 3],
        [0, 2, 3],
    ]

    goals = [
        [10, 2, 3],
        [10, 1, 3],
        [10, 0, 3],
    ]

    print("=== Standard Hungarian ===")
    assignment1 = allocator.allocate(drones, goals)

    print("\n=== Fair Hungarian (Min-Max) ===")
    assignment2 = allocator.allocate_fair(drones, goals)
