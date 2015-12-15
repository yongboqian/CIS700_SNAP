#!/usr/bin/env bash

if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <LABEL> <NUM_STAGES> <FEATURE_TYPE>"
    exit 1
fi

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
CREATE_SAMPLES="${SCRIPT_DIR}/createsamples.pl"
MERGE_VEC="${SCRIPT_DIR}/mergevec.py"
OCV_BIN="/opt/opencv3/bin"
OCV_BIN="/opt/opencv3.0.0-master/bin"
#OCV_BIN="/data/v52/nstiurca/codes/opencv-3.0.0-beta-Release/bin"
OCV_CREATE_SAMPLES="${OCV_BIN}/opencv_createsamples"
OCV_TRAIN_CASCADE="${OCV_BIN}/opencv_traincascade"

LABEL="$1"
IMAGES_DIR="images"
TIGHT="tight_"
CLASSIFIER="classifier_"
EXTS="png jpg JPG jpeg"
WIDTH=16
HEIGHT=16
NUM_POS_TOTAL=5400
BGCOLOR=0
BGTHRESH=0
MAX_X_ANGLE=1.1
MAX_Y_ANGLE=1.1
MAX_Z_ANGLE=0.5
SCALE=16.0
MAX_I_DEV=40
BUF_SIZE=1024
FEATURE_TYPE=$3 #HAAR LBP or HOG
NUM_POS=4000
NUM_NEG=1000
NUM_STAGES=$2
BT=GAB # DAR RAB LB or GAB
MIN_HIT_RATE=0.999
MAX_FALSE_ALARM_RATE=0.5
WEIGHT_TRIM_RATE=0.95
MAX_DEPTH=1
MAX_WEAK_COUNT=100
MODE=ALL

# prepare text files
FG="${TIGHT}${LABEL}.txt"
BG="not_${FG}"
echo Preparing text files $FG and $BG
if [ -f $FG ] ; then
    echo $FG already exists. Delete if you want to re-create it
else
    for EXT in $EXTS; do
        ls ${IMAGES_DIR}/${TIGHT}segmented_${LABEL}_*.$EXT >>"$FG"
    done
fi
if [ -f $BG ] ; then
    echo $BG already exists. delet if you want to re-create it
else
    for EXT in $EXTS; do
        ls ${IMAGES_DIR}/*.$EXT | grep -v cropped | grep -v segmented | grep -v "$LABEL" >> "$BG"
    done
fi

# create samples
VECS_DIR="${TIGHT}${LABEL}_vecs_${WIDTH}x${HEIGHT}/"
echo
if [ -d $VECS_DIR ] ; then
    echo $VECS_DIR already exists. Delete it if you want to re-create it
else
    echo Creating samples to $VECS_DIR
    echo $CREATE_SAMPLES "$FG" "$BG" "$VECS_DIR" $NUM_POS_TOTAL \
        "$OCV_CREATE_SAMPLES -bgcolor $BGCOLOR -bgthresh $BGTHRESH -w $WIDTH -h $HEIGHT \
        -maxxangle $MAX_X_ANGLE -maxyangle $MAX_Y_ANGLE -maxzangle $MAX_Z_ANGLE"
    $CREATE_SAMPLES "$FG" "$BG" "$VECS_DIR" $NUM_POS_TOTAL \
        "$OCV_CREATE_SAMPLES -bgcolor $BGCOLOR -bgthresh $BGTHRESH -w $WIDTH -h $HEIGHT \
        -maxxangle $MAX_X_ANGLE -maxyangle $MAX_Y_ANGLE -maxzangle $MAX_Z_ANGLE"

    # view samples
    # TODO: optional?
    SAMPLE_VEC="`ls "$VECS_DIR" | head -1`"
    echo
    echo Here is $SAMPLE_VEC
    echo "================================================================="
    echo "======================= PRESS ESCAPE ============================"
    echo "================================================================="
    echo $OCV_CREATE_SAMPLES -vec "${VECS_DIR}/${SAMPLE_VEC}" -w $WIDTH -h $HEIGHT -show scale = $SCALE
    $OCV_CREATE_SAMPLES -vec "${VECS_DIR}/${SAMPLE_VEC}" -w $WIDTH -h $HEIGHT -show scale = $SCALE
fi

# merge vecs
MERGED_VEC="${TIGHT}${LABEL}_${WIDTH}x${HEIGHT}.vec"
echo
if [ -f $MERGED_VEC ] ; then
    echo $MERGED_VEC already exists. Delete it if you want to re-create it
else
    echo Merging vecs to $MERGED_VEC
    echo $MERGE_VEC -v "$VECS_DIR" -o "$MERGED_VEC"
    $MERGE_VEC -v "$VECS_DIR" -o "$MERGED_VEC"
fi

# train
CLASSIFIER_DIR="${CLASSIFIER}${TIGHT}${LABEL}_${WIDTH}x${HEIGHT}_${FEATURE_TYPE}"
mkdir -p "$CLASSIFIER_DIR"
echo
echo Training $CLASSIFIER_DIR
echo $OCV_TRAIN_CASCADE -vec "$MERGED_VEC" -bg "$BG" -precalcValBufSize $BUF_SIZE -precalcIdxBufSize $BUF_SIZE \
    -numPos $NUM_POS -numNeg $NUM_NEG -numStages $NUM_STAGES -minHitRate $MIN_HIT_RATE -bt $BT \
    -maxFalseAlarmRate $MAX_FALSE_ALARM_RATE -w $WIDTH -h $HEIGHT -mode $MODE -maxWeakCount $MAX_WEAK_COUNT \
    -featureType $FEATURE_TYPE -maxDepth $MAX_DEPTH -weightTrimRate $WEIGHT_TRIM_RATE -data "$CLASSIFIER_DIR" \
    "&&" cp "${CLASSIFIER_DIR}/cascade.xml" "${CLASSIFIER_DIR}.xml"
$OCV_TRAIN_CASCADE -vec "$MERGED_VEC" -bg "$BG" -precalcValBufSize $BUF_SIZE -precalcIdxBufSize $BUF_SIZE \
    -numPos $NUM_POS -numNeg $NUM_NEG -numStages $NUM_STAGES -minHitRate $MIN_HIT_RATE -bt $BT \
    -maxFalseAlarmRate $MAX_FALSE_ALARM_RATE -w $WIDTH -h $HEIGHT -mode $MODE -maxWeakCount $MAX_WEAK_COUNT \
    -featureType $FEATURE_TYPE -maxDepth $MAX_DEPTH -weightTrimRate $WEIGHT_TRIM_RATE -data "$CLASSIFIER_DIR" \
    && cp "${CLASSIFIER_DIR}/cascade.xml" "${CLASSIFIER_DIR}.xml"

echo
echo Bon apepit
echo

