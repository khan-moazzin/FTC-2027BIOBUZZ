"""Smoke-test a CSV with the pinned upstream AdvantageScope decoder (requires Node and network).

Usage: python tools/verify_ascope_csv.py TeamCode/build/logging-fixtures/synthetic-ascope.csv
The small log stub captures decoded values; this does not test the desktop UI.
"""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import urllib.request

REVISION = "abb616bdd575dfa27f5794ed6be427ffd0209897"
URL = f"https://raw.githubusercontent.com/Mechanical-Advantage/AdvantageScope/{REVISION}/src/hub/dataSources/csv/CSVDecoder.ts"
source = urllib.request.urlopen(URL, timeout=30).read().decode("utf-8")
# Erase only the three known TypeScript declarations; execute the original decoder logic.
source = source.replace('import Log from "../../../shared/log/Log";', "")
source = source.replace("export default class CSVDecoder", "class CSVDecoder")
source = source.replace("private parseEntry(log: Log, timestamp: number, key: string, valueRaw: string)", "parseEntry(log, timestamp, key, valueRaw)")
source = source.replace("decode(log: Log, data: string, progressCallback?: (progress: number) => void)", "decode(log, data, progressCallback)")
source += r'''
const fs = require("fs");
const entries = [];
const log = {
  putUnknownStruct(key, time, value) { entries.push({key, time, value}); },
  putRaw(key, time, value) { entries.push({key, time, value: Array.from(value)}); }
};
new CSVDecoder().decode(log, fs.readFileSync(process.argv[2], "utf8"));
if (!entries.length || entries.some(e => !Number.isFinite(e.time))) throw Error("Invalid/empty decoded timeline");
if (entries.some(e => e.key.startsWith('"'))) throw Error("Quoted key leaked into field names");
const reason = entries.find(e => e.key === "Test/Reason");
if (reason && reason.value !== 'Rejected, "pose"\nretry') throw Error("String round-trip failed");
console.log(JSON.stringify({entries: entries.length, fields: [...new Set(entries.map(e => e.key))].length, first: entries[0]}));
'''
with tempfile.TemporaryDirectory(prefix="biobuzz-ascope-") as temporary:
    script = Path(temporary) / "decoder-smoke.cjs"
    script.write_text(source, encoding="utf-8")
    subprocess.run(["node", str(script), str(Path(sys.argv[1]).resolve())], check=True)
print("PASS: upstream CSVDecoder", REVISION)
