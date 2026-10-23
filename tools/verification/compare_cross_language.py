"""Report every numeric/discrete difference; never retune robot acceptance.

Port equivalence is established by dedicated sensor/command tests and unchanged
scenario gates. This diagnostic reports observed closed-loop roundoff, including
unfavorable changes, without inventing a new pass threshold after seeing results.
"""
from pathlib import Path
import argparse
import csv
import json
import math
import re

def compare(a,b,path,stats,discrete):
    if isinstance(a,dict) and isinstance(b,dict):
        if a.keys()!=b.keys():discrete.append({"path":path,"before_keys":list(a),"after_keys":list(b)})
        for key in a.keys() & b.keys():compare(a[key],b[key],path+"."+key,stats,discrete)
    elif isinstance(a,list) and isinstance(b,list):
        if len(a)!=len(b):discrete.append({"path":path,"lengths":[len(a),len(b)]})
        for index,(x,y) in enumerate(zip(a,b)):compare(x,y,f"{path}[{index}]",stats,discrete)
    elif isinstance(a,(float,int)) and isinstance(b,(float,int)) and not isinstance(a,bool) and not isinstance(b,bool):
        key=re.sub(r"\[\d+\]","[]",path)
        row=stats.setdefault(key,{"count":0,"unequal":0,"max_abs_delta":0.,"max_delta_path":None})
        row["count"]+=1
        if a!=b:
            row["unequal"]+=1;delta=abs(a-b)
            if not math.isfinite(delta):discrete.append({"path":path,"values":[a,b]})
            if delta>row["max_abs_delta"]:row["max_abs_delta"]=delta;row["max_delta_path"]=path
            if isinstance(a,int) and isinstance(b,int):discrete.append({"path":path,"values":[a,b]})
    elif a!=b:discrete.append({"path":path,"values":[a,b]})

def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--session-dir",type=Path,required=True)
    cli.add_argument("--native-baseline",default="native_baseline_final")
    cli.add_argument("--native-radial",default="native_radial_final")
    cli.add_argument("--comparison-name",default="comparison_final")
    args=cli.parse_args();root=args.session_dir.resolve();output=root/args.comparison_name;output.mkdir(exist_ok=True)
    pairs=[("baseline/"+name,root/"python_baseline"/name,root/args.native_baseline/name) for name in (
        "stage3_moving_q_off.json","stage3_moving_history.json","stage4_final_baseline_metrics.json","stage4_final_baseline_history.json")]
    # Match all fields, including discrete event/count changes. Keep path and
    # measured planner-time differences visible in the separate differences file.
    pairs.append(("radial",root/"python_radial/stage9_control.json",root/args.native_radial/"stage9_control.json"))
    report={}
    for name,old,new in pairs:
        stats={};discrete=[];compare(json.loads(old.read_text()),json.loads(new.read_text()),"root",stats,discrete)
        filename=name.replace("/","_").replace(".json","")
        (output/(filename+"_fields.json")).write_text(json.dumps(stats,indent=2))
        (output/(filename+"_discrete.json")).write_text(json.dumps(discrete,indent=2))
        report[name]={"numeric_values":sum(s["count"] for s in stats.values()),
            "unequal_numeric_values":sum(s["unequal"] for s in stats.values()),
            "max_abs_numeric_delta":max((s["max_abs_delta"] for s in stats.values()),default=0),
            "discrete_or_path_differences":len(discrete),
            "discrete_differences_file":filename+"_discrete.json"}
    history=json.loads((root/"python_baseline/stage3_moving_history.json").read_text())
    native=json.loads((root/args.native_baseline/"stage3_moving_history.json").read_text())
    fields=("v_hat_m_s","v_ref_m_s","theta_hat_rad","actual_sum_nm")
    history_deltas={key:max(abs(a[key]-b[key]) for a,b in zip(history,native)) for key in fields if key in history[0]}
    report["moving_payload_history_max_abs_delta"]=history_deltas
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(9,7),sharex=True)
    for axis,key in zip(axes,("v_hat_m_s","actual_sum_nm","v_ref_m_s")):
        axis.plot([row["t"] for row in history],[row[key] for row in history],label="Python reference")
        axis.plot([row["t"] for row in native],[row[key] for row in native],"--",label="Shared C")
        axis.set_ylabel(key);axis.grid(True,alpha=.3)
    axes[0].legend();axes[-1].set_xlabel("Simulation time (s)")
    fig.tight_layout();fig.savefig(output/"moving_payload_overlay.png",dpi=160);plt.close(fig)
    (output/"summary.json").write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=="__main__":main()
