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
  RAW_MODEL=$(tr -d '\0' < /sys/firmware/devicetree/base/model 2>/dev/null || true)
  if echo "$RAW_MODEL" | grep -qi "tici"; then
    export AGNOS_VERSION="12.8"
  else
    export AGNOS_VERSION="16"
  fi
fi

export STAGING_ROOT="/data/safe_staging"
