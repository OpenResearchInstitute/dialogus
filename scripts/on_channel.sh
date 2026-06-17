#!/bin/sh

# This lets you run Dialogus on a specified channel number,
# using the channel map from Bouro.
#
# syntax:
# on_channel.sh 37 ./dialogus_11e05ec some other args
# is equivalent to
# ./dialogus_11e05ec some other args FREQ=5604062500

basefreq=5600000000  # Out of band!

f4chan() {
    local basefreq=$1
    local chan=$2
    local freq

    if [ "$chan" -lt 0 ]; then
        echo "No negative channel numbers!" >&2
        return 1
    elif [ "$chan" -lt 32 ]; then
        freq=$((basefreq - (chan+1)*156250))
        echo "$freq"
    elif [ "$chan" -eq 32 ]; then
        echo "32 is not a valid channel" >&2
        return 1
    elif [ "$chan" -lt 64 ]; then
        freq=$((basefreq + (63-chan)*156250))
        echo "$freq"
    else
        echo "No channel numbers greater than 63!" >&2
        return 1
    fi
}

if [ "$#" -lt 2 ]; then
    echo "Run Dialogus on a specified channel number (Bouro channels)"
    echo "Syntax:"
    echo "    on_channel.sh <channelnumber> <path-to-dialogus> [<optional other args>]"
    exit 1
fi

# Check that the channel number is not empty and contains ONLY digits
case "$1" in
    ''|*[!0-9]*) 
        echo "Error: '$1' is not a valid channel number." >&2
        exit 1 
        ;;
esac
channel=$(( $1 ))

echo "channel = $channel"

if [[ ! -f "$2" ]]; then
    echo "$2 does not exist."
    exit 1
fi

if [[ ! -x "$2" ]]; then
    echo "$2 is not executable. Check permissions."
    exit 1
fi

frequency=$(f4chan "$basefreq" "$channel") || exit 1

# Build command line
cmd="$(realpath "$2")"
shift 2
if [ "$#" -gt 0 ]; then
    cmd="$cmd $@ FREQ=$frequency"
else
    cmd="$cmd FREQ=$frequency"
fi

echo "$cmd"

# Run the command
/bin/sh -c "$cmd"
