#!/bin/bash
# Fetch the ORB vocabulary file that ORB_SLAM3's build.sh expects at
# src/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz, then extract it to ORBvoc.txt.
#
# The file isn't tracked in this repo (~125MB) or vendored into the
# ORB_SLAM3 fork under src/ - this script downloads it straight from the
# upstream UZ-SLAMLab/ORB_SLAM3 repo instead.
#
# Usage: scripts/setup/fetch_orb_vocabulary.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VOCAB_DIR="$REPO_ROOT/src/ORB_SLAM3/Vocabulary"
VOCAB_URL="https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz"

if [ ! -d "$VOCAB_DIR" ]; then
    echo "ERROR: $VOCAB_DIR does not exist - clone src/ORB_SLAM3 first." >&2
    exit 1
fi

if [ -f "$VOCAB_DIR/ORBvoc.txt" ]; then
    echo "ORBvoc.txt already present at $VOCAB_DIR/ORBvoc.txt - nothing to do."
    exit 0
fi

if [ ! -f "$VOCAB_DIR/ORBvoc.txt.tar.gz" ]; then
    echo "Downloading ORB vocabulary from upstream ORB_SLAM3..."
    curl -fL --retry 3 -o "$VOCAB_DIR/ORBvoc.txt.tar.gz" "$VOCAB_URL"
fi

echo "Extracting ORBvoc.txt.tar.gz..."
tar -xf "$VOCAB_DIR/ORBvoc.txt.tar.gz" -C "$VOCAB_DIR"

echo "Done: $VOCAB_DIR/ORBvoc.txt"
