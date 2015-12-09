#!/usr/bin/env bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <label>"
    exit 1
fi

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
CREATE_SAMPLES="${SCRIPT_DIR}/createsamples.pl"
MERGE_VEC="${SCRIPT_DIR}/mergevec.py"
OCV_BIN="/opt/opencv3/bin"
OCV_CREATE_SAMPLES="${OCV_BIN}/opencv_createsamples"
OCV_TRAIN_CASCADE="${OCV_BIN}/opencv_traincascade"

LABEL="$1"
IMAGES_DIR="images"
TIGHT="tight_"
CLASSIFIER="classifier_"
EXT="png"
WIDTH=32
HEIGHT=32
NUM_POS_TOTAL=4500
BGCOLOR=0
BGTHRESH=0
MAX_X_ANGLE=1.1
MAX_Y_ANGLE=1.1
MAX_Z_ANGLE=0.5
MAX_I_DEV=40
BUF_SIZE=1024
FEATURE_TYPE=HAAR #HAAR LBP or HOG
NUM_POS=1000
NUM_NEG=1000
NUM_STAGES=40
BT=GAB # DAR RAB LB or GAB
MIN_HIT_RATE=0.995
MAX_FALSE_ALARM_RATE=0.5
WEIGHT_TRIM_RATE=0.95
MAX_DEPTH=1
MAX_WEAK_COUNT=100
MODE=ALL

# prepare text files
FG="${TIGHT}${LABEL}.txt"
BG="not_${FG}"
echo Preparing text files $FG and $BG
ls ${IMAGES_DIR}/${TIGHT}segmented_${LABEL}*.$EXT > "$FG"
ls ${IMAGES_DIR}/*.$EXT | grep -v cropped | grep -v segmented | grep -v "$LABEL" > "$BG"

# create samples
VECS_DIR="${TIGHT}${LABEL}_vecs_${WIDTH}x${HEIGHT}/"
echo
echo Creating samples to $VECS_DIR
$CREATE_SAMPLES "$FG" "$BG" "$VECS_DIR" $NUM_POS_TOTAL \
    "$OCV_CREATE_SAMPLES -bgcolor $BGCOLOR -bgthresh $BGTHRESH -w $WIDTH -h $HEIGHT"

# view samples
# TODO: optional?
SAMPLE_VEC="`ls "$VECS_DIR" | head -1`"
echo
echo Here is $SAMPLE_VEC
echo "================================================================="
echo "======================= PRESS ESCAPE ============================"
echo "================================================================="
$OCV_CREATE_SAMPLES -vec "${VECS_DIR}/${SAMPLE_VEC}" -w $WIDTH -h $HEIGHT

# merge vecs
MERGED_VEC="${TIGHT}${LABEL}_${WIDTH}x${HEIGHT}.vec"
echo
echo Merging vecs to $MERGED_VEC
$MERGE_VEC -v "$VECS_DIR" -o "$MERGED_VEC"

# train
CLASSIFIER_DIR="${CLASSIFIER}${TIGHT}${LABEL}_${WIDTH}x${HEIGHT}_${FEATURE_TYPE}"
mkdir -p "$CLASSIFIER_DIR"
echo
echo Training $CLASSIFIER_DIR
$OCV_TRAIN_CASCADE -vec "$MERGED_VEC" -bg "$BG" -precalcValBufSize $BUF_SIZE -precalcIdxBufSize $BUF_SIZE \
    -numPos $NUM_POS -numNeg $NUM_NEG -numStages $NUM_STAGES -minHitRate $MIN_HIT_RATE -bt $BT \
    -maxFalseAlarmRate $MAX_FALSE_ALARM_RATE -w $WIDTH -h $HEIGHT -mode $MODE -maxWeakCount $MAX_WEAK_COUNT \
    -featureType $FEATURE_TYPE -maxDepth $MAX_DEPTH -weightTrimRate $WEIGHT_TRIM_RATE -data "$CLASSIFIER_DIR"

echo
echo Bon apepit
echo

