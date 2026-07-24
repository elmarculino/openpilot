#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="18.5"
fi

export STAGING_ROOT="/data/safe_staging"

# Haval H6 PHEV 2026 (MK4): OBD/FW query returns 0 ECUs on this harness, so force the platform.
# Also have CAN FINGERPRINTS in opendbc gwm/fingerprints.py as a fallback without this env.
export FINGERPRINT=GWM_HAVAL_H6_MK4
export SKIP_FW_QUERY=1
