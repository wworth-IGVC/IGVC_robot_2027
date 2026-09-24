#!/usr/bin/env bash
# Download the YOLOPv2 TorchScript weights into $REPO_ROOT/models/yolopv2.pt.
#
# The weights (MIT-licensed) come from the upstream CAIC-AD/YOLOPv2 GitHub
# release and are intentionally kept out of version control (see .gitignore).
# Run this once per machine:
#
#     ./src/igvc_lane_detection/scripts/fetch_yolopv2_weights.sh
#     export YOLOPV2_WEIGHTS=$PWD/models/yolopv2.pt
#
# Jetson AGX Orin users: PyTorch itself must be installed separately from
# the NVIDIA JetPack wheel index — see training/README.md.
#
# The file is checked against a SHA256 before it is used, both after a
# download and when it is already present. Upstream publishes no checksum for
# this 2022 release, so the value below was recorded on 2026-09-24 from a
# download whose size, 156380200 bytes, matches the size the GitHub API
# reports for the release asset, and which torch.jit.load opens. See
# docs/GAZEBO_AGX_BASELINE_REPORT.md section 20.

set -euo pipefail

URL="https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt"
EXPECTED_SHA256="f2a8c8374203ae3e67ff9c184e931f763957de92a993b23269e4e721627f1f8c"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Script lives at <repo>/src/igvc_lane_detection/scripts/
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
DEST_DIR="${REPO_ROOT}/models"
DEST_FILE="${DEST_DIR}/yolopv2.pt"
PART_FILE="${DEST_FILE}.part"

# sha256sum on Linux and Git Bash, shasum on macOS.
sha256_of() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    echo "ERROR: neither sha256sum nor shasum is installed, so the weights cannot be verified." >&2
    return 2
  fi
}

# Returns 0 when the file matches EXPECTED_SHA256, and says which way it went.
checksum_ok() {
  local got
  got="$(sha256_of "$1")" || return 2
  if [[ "${got}" == "${EXPECTED_SHA256}" ]]; then
    echo "SHA256 OK: ${got}"
    return 0
  fi
  echo "SHA256 MISMATCH for $1" >&2
  echo "  expected ${EXPECTED_SHA256}" >&2
  echo "  got      ${got}" >&2
  return 1
}

mkdir -p "${DEST_DIR}"

if [[ -f "${DEST_FILE}" ]]; then
  if checksum_ok "${DEST_FILE}"; then
    echo "yolopv2.pt already present and verified at ${DEST_FILE} — skipping."
    exit 0
  fi
  # Not deleted automatically: it may be a deliberately different model.
  echo "ERROR: ${DEST_FILE} is not the upstream YOLOPv2 release." >&2
  echo "  If it is a download that went wrong, delete it and rerun this script." >&2
  echo "  If it is deliberately a different model, keep it somewhere else and" >&2
  echo "  point YOLOPV2_WEIGHTS at it instead of this path." >&2
  exit 1
fi

rm -f "${PART_FILE}"
echo "Downloading YOLOPv2 weights → ${DEST_FILE}"
if command -v curl >/dev/null 2>&1; then
  curl --fail --location --progress-bar -o "${PART_FILE}" "${URL}"
elif command -v wget >/dev/null 2>&1; then
  wget --show-progress -O "${PART_FILE}" "${URL}"
else
  echo "ERROR: neither curl nor wget is installed." >&2
  exit 1
fi

# Verify before the file takes the name the node loads, so a truncated or
# altered download can never be picked up by lane_segmentation_node.
if ! checksum_ok "${PART_FILE}"; then
  mv -f "${PART_FILE}" "${DEST_FILE}.rejected"
  echo "ERROR: the download did not match; kept as ${DEST_FILE}.rejected for inspection." >&2
  exit 1
fi
mv -f "${PART_FILE}" "${DEST_FILE}"

echo
echo "Done. Export the path so the launch file picks it up:"
echo "  export YOLOPV2_WEIGHTS=\"${DEST_FILE}\""
