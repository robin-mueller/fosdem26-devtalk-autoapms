# Settings
TITLE="fosdem26_$1"
OUTPUT_FILE="$1.cast"

# Record the terminal session
export COMMAND_TO_RUN="$2"
asciinema rec $OUTPUT_FILE -c "tmuxinator start -p start_tmux.yml" -t "$TITLE" --overwrite