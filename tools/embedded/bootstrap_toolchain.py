"""Install Ubuntu Noble's ARM GNU packages locally, with no system/root install."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess

ROOT=Path(__file__).resolve().parents[2]
PACKAGES={"gcc-arm-none-eabi":"15:13.2.rel1-2", "binutils-arm-none-eabi":"2.42-1ubuntu1+23",
          "libnewlib-arm-none-eabi":"4.4.0.20231231-2", "libnewlib-dev":"4.4.0.20231231-2"}

def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--download",action="store_true",help="Download pinned packages from configured Ubuntu apt sources")
    args=cli.parse_args()
    base=ROOT/".venv/embedded";debs=base/"debs";native=base/"native"
    debs.mkdir(parents=True,exist_ok=True)
    if args.download:
        subprocess.run(["apt-get","download",*[f"{p}={v}" for p,v in PACKAGES.items()]],cwd=debs,check=True)
    record={}
    for name,version in PACKAGES.items():
        candidates=[]
        for path in debs.glob(name+"_*.deb"):
            actual=subprocess.check_output(["dpkg-deb","-f",str(path),"Version"],text=True).strip()
            if actual==version:candidates.append(path)
        if len(candidates)!=1:raise SystemExit(f"Need exactly one {name}={version}; run --download")
        path=candidates[0]
        subprocess.run(["dpkg-deb","-x",str(path),str(native)],check=True)
        record[name]={"version":version,"file":path.name,"sha256":hashlib.sha256(path.read_bytes()).hexdigest()}
    prefix=native/"usr/lib/arm-none-eabi"
    for name,target in (("include","../../include/newlib"),("lib","newlib")):
        link=prefix/name
        if not link.exists() and not link.is_symlink():link.symlink_to(target)
    gcc=native/"usr/bin/arm-none-eabi-gcc"
    record["gcc_version"]=subprocess.check_output([str(gcc),"--version"],text=True)
    (base/"toolchain_manifest.json").write_text(json.dumps(record,indent=2))
    print(record["gcc_version"]);print(f"Local toolchain: {gcc}")

if __name__=="__main__":main()
