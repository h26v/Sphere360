#!/usr/bin/env bash
# Gọi API debug của Sphere360 (debug.DebugActivity) qua adb rồi kéo kết quả về $WORK/s<giờ lượt>/.
#   bash tools/dbg.sh install                       cài APK vừa build
#   bash tools/dbg.sh params                        in danh sách tham số bộ ghép
#   bash tools/dbg.sh stitch <lượt> <nhãn> [params] [width]
#       vd: bash tools/dbg.sh stitch 20261023_171607 w4 "kSeamParallaxWeight=4,kDebugSeams=1"
# Biến môi trường: ADB (mặc định adb trên PATH), WORK (mặc định ./work).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(dirname "$HERE")
WORK=${WORK:-$PWD/work}
A=${ADB:-adb}
APP=/sdcard/Android/data/com.sphere360.app/files
export MSYS_NO_PATHCONV=1
ACT=com.sphere360.app/.debug.DebugActivity

case "$1" in
install)
    $A install -r "$REPO/app/build/outputs/apk/release/app-release.apk" | tail -1
    ;;
params)
    $A shell rm -f $APP/params.txt
    $A shell am start -n $ACT --es cmd params >/dev/null
    for i in $(seq 1 20); do sleep 0.5; $A shell cat $APP/params.txt 2>/dev/null && break; done
    ;;
stitch)
    SID=$2; TAG=$3; PARAMS=${4:-kDebugSeams=1}; WIDTH=${5:-8192}
    S=$APP/sessions/$SID
    OUT=$WORK/s${SID:9:4}
    mkdir -p "$OUT"
    $A shell rm -f $S/$TAG.done
    $A shell am start -n $ACT --es cmd stitch --es session $SID --es out $TAG.jpg --ei width $WIDTH \
        --es params "'$PARAMS'" >/dev/null
    START=$(date +%s)
    for i in $(seq 1 120); do
        sleep 3
        if $A shell ls $S/$TAG.done >/dev/null 2>&1; then break; fi
        if $A logcat -d -s AndroidRuntime:E | grep -q "FATAL.*sphere360"; then echo "app crash"; exit 1; fi
    done
    echo "chờ $(( $(date +%s) - START )) giây"
    $A shell cat $S/$TAG.done; echo
    for f in $TAG.log $TAG.jpg ${TAG}_seams.jpg $($A shell ls $S | tr -d '' | grep "^${TAG}_chain_"); do
        $A pull $S/$f "$OUT/$f" >/dev/null 2>&1
    done
    grep -v "^Cặp [0-9]\|^W Bỏ cặp" "$OUT/$TAG.log" | tail -6
    python "$HERE/grid_views.py" "$OUT/$TAG.jpg" "$OUT/$TAG"
    ;;
*)
    sed -n 2,6p "$0"
    ;;
esac
