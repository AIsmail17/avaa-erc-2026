#!/usr/bin/env bash
# Send a question, and optionally a file, to a local model served by LM Studio.
#
#     tools/ask-local.sh "what state transitions happened?" /tmp/sim.log
#     tools/ask-local.sh "summarise the errors" < some.log
#     tools/ask-local.sh --models
#
# LM Studio's server speaks the OpenAI API at http://localhost:1234/v1, so this is just
# curl. Start it in LM Studio under Developer -> Start Server, with a model loaded.
#
# What this is for
# ----------------
# Not for the debugging. A 14-32B local model will produce a confident wrong analysis of
# something like a four-bar linkage offset, and checking it costs more than doing it. What
# it is genuinely good at here is the bulk work that would otherwise be read line by line:
#
#     tools/ask-local.sh "list every state transition and every ERROR, in order" /tmp/sim.log
#     tools/ask-local.sh "did the reach complete? quote the lines that say so" /tmp/run_raw.log
#
# A simulation log runs to tens of thousands of lines and most of it is controller chatter.
# Triaging that locally costs nothing and no waiting.
set -u

HOST="${LOCAL_AI_HOST:-http://localhost:1234/v1}"
MODEL="${LOCAL_AI_MODEL:-}"
MAX_CHARS="${LOCAL_AI_MAX_CHARS:-120000}"

if [ "${1:-}" = "--models" ]; then
    curl -s -m 10 "$HOST/models" | python3 -c '
import json,sys
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit("no answer from '"$HOST"' -- is the LM Studio server started?")
for m in d.get("data", []):
    print("  " + m.get("id", "?"))
' || exit 1
    exit 0
fi

[ $# -ge 1 ] || { sed -n '2,8p' "$0"; exit 1; }
QUESTION="$1"; shift

if [ $# -ge 1 ]; then
    CONTENT=$(tail -c "$MAX_CHARS" "$1")
    SOURCE="$1"
else
    CONTENT=$(tail -c "$MAX_CHARS")
    SOURCE="stdin"
fi

if [ -z "$MODEL" ]; then
    MODEL=$(curl -s -m 10 "$HOST/models" \
        | python3 -c 'import json,sys; print(json.load(sys.stdin)["data"][0]["id"])' \
        2>/dev/null) || true
fi
[ -n "$MODEL" ] || { echo "no model loaded; open LM Studio, load one, start the server" >&2; exit 1; }

# The payload is built in python so the log content is escaped properly rather than
# fighting shell quoting, which has already cost this project a corrupted source file.
printf '%s' "$CONTENT" | python3 -c '
import json, sys, urllib.request
host, model, question, source = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
body = sys.stdin.read()
payload = {
    "model": model,
    "messages": [
        {"role": "system",
         "content": "You are triaging robot simulation logs. Answer only from the text "
                    "given. Quote exact lines as evidence. If the text does not say, say "
                    "so rather than inferring."},
        {"role": "user",
         "content": "%s\n\n--- contents of %s ---\n%s" % (question, source, body)},
    ],
    "temperature": 0.2,
    "stream": False,
}
req = urllib.request.Request(
    host + "/chat/completions",
    data=json.dumps(payload).encode(),
    headers={"Content-Type": "application/json"})
try:
    with urllib.request.urlopen(req, timeout=600) as r:
        out = json.load(r)
except Exception as exc:
    sys.exit("local model did not answer: %s" % exc)
print(out["choices"][0]["message"]["content"])
' "$HOST" "$MODEL" "$QUESTION" "$SOURCE"
