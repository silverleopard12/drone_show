# Archived Documentation

This directory contains historical documentation that has been archived but may still be useful for reference.

## Files

### CLEANUP_PLAN.md
- **Date**: 2025-11-19
- **Status**: Completed
- **Purpose**: Plan for cleaning up redundant files and scripts
- **Note**: Cleanup has been completed. This file is kept for historical reference.

### OFFLINE_TRAJECTORY_PLANNING.md
- **Date**: 2025-11-14 to 2025-11-15
- **Status**: Resolved (planning_horizon=70m recommended, 94% success rate)
- **Purpose**: Documentation of offline trajectory planning experiments
- **Key Findings**:
  - Planning horizon affects crash rate significantly
  - 70m horizon optimal for 36-drone swarm
  - Detailed analysis of LBFGS optimization issues

### TRAJECTORY_SAMPLING.md
- **Date**: 2025-11-18
- **Status**: Functional guide
- **Purpose**: Comprehensive guide for trajectory sampling using `sample_trajectories.py`
- **Contents**:
  - Multiple output formats (CSV, JSON, NumPy, DroneShow)
  - Data analysis examples
  - Performance metrics

### QUICK_SAMPLING_GUIDE.md
- **Date**: 2025-11-18
- **Status**: Quick reference
- **Purpose**: Quick start guide for 25-drone trajectory sampling
- **Note**: Subset of TRAJECTORY_SAMPLING.md, kept for quick reference

## Why Archived?

These documents were moved to archive because:
1. **CLEANUP_PLAN.md**: Task completed, kept for historical reference
2. **OFFLINE_TRAJECTORY_PLANNING.md**: Experimental phase completed, findings integrated into main codebase
3. **TRAJECTORY_SAMPLING.md**: Superseded by integrated trajectory recording system
4. **QUICK_SAMPLING_GUIDE.md**: Duplicate content, simplified version of main guide

## Using Archived Documentation

While these documents are archived, they may still contain useful information:
- Technical details about optimization parameters
- Debugging procedures
- Historical context for design decisions
- Experimental results and analysis

If you need to reference these documents, they remain available in this directory.

---

**Last Updated**: 2025-11-20
