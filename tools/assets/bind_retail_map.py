"""Bind canonical venue poses to a real saved map; official Action proof only.

Candidates remain unverified. A successful ComputePathToPose report for the
same map and coordinates can promote them; no path search is implemented here.
"""
import argparse
import hashlib
import json
from pathlib import Path

import yaml
from companionbot_contracts.paths import repository_root
from companionbot_behavior.core.catalog import JsonCatalogProvider


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    root=repository_root();cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--map-yaml",type=Path,required=True);cli.add_argument("--output",type=Path,required=True)
    cli.add_argument("--destinations",type=Path,default=root/"models/minisegway/retail_navigation/config/map_destinations.json")
    cli.add_argument("--catalog",type=Path,default=root/"models/minisegway/retail_navigation/config/catalog.json")
    cli.add_argument("--proof-dir",type=Path);args=cli.parse_args()
    if args.output.exists():raise FileExistsError("refuse to overwrite an existing map binding")
    spec=json.loads(args.destinations.read_text());catalog=JsonCatalogProvider.load(args.catalog)
    if set(spec["destinations"]) != set(catalog.destination_ids()):raise ValueError("venue_ids_and_map_targets_differ")
    lock=json.loads((root/"models/minisegway/retail_navigation/bookstore_world.lock.json").read_text())
    if spec["world_commit"]!=lock["commit"]:raise ValueError("destination_world_version_mismatch")
    map_yaml=args.map_yaml.resolve();metadata=yaml.safe_load(map_yaml.read_text());image=(map_yaml.parent/metadata["image"]).resolve()
    yaml_hash,image_hash=digest(map_yaml),digest(image)
    binding=dict(schema_version=1,map_id=spec["map_id"],map_version=hashlib.sha256((yaml_hash+image_hash).encode()).hexdigest()[:16],
        map_yaml=str(map_yaml),map_sha256=yaml_hash,image_sha256=image_hash,
        catalog_revision=catalog.revision,catalog_source=str(args.catalog.resolve()),catalog_sha256=digest(args.catalog),
        destination_config=str(args.destinations.resolve()),destination_config_sha256=digest(args.destinations),
        world_commit=spec["world_commit"],map_frame_contract=spec["map_frame_contract"],
        destinations={name:dict(target,reachability_verified=False) for name,target in spec["destinations"].items()})
    if args.proof_dir:
        proof=json.loads((args.proof_dir/"summary.json").read_text());tested=json.loads((args.proof_dir/"bindings.json").read_text())
        if not proof["passed"] or any(tested[k]!=binding[k] for k in ("map_id","map_version","map_sha256","image_sha256")):
            raise ValueError("planning_proof_map_mismatch")
        reports={r["destination_id"]:r for r in proof["results"]}
        for name,target in binding["destinations"].items():
            previous=tested["destinations"][name]
            if any(previous[k]!=target[k] for k in ("x_m","y_m","yaw_rad")):
                raise ValueError("planning_proof_target_mismatch:"+name)
            result=reports[name]
            if result["status"]!=4 or not result["path"]["poses"]:raise ValueError("destination_not_reachable:"+name)
            target["reachability_verified"]=True
        binding["reachability_evidence"]=str((args.proof_dir/"summary.json").resolve())
        binding["reachability_evidence_sha256"]=digest(args.proof_dir/"summary.json")
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(binding,indent=2,ensure_ascii=False)+"\n")
    print(json.dumps(dict(output=str(args.output.resolve()),map_id=binding["map_id"],map_version=binding["map_version"],
        verified=all(t["reachability_verified"] for t in binding["destinations"].values()))),flush=True)


if __name__=="__main__":main()
