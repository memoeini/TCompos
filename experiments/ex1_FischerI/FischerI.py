# Run the script with an integer argument n to generate a model with n processes

import sys

def acq_upper_bound(n):
    return 50 + 10 * n

def controller_thresholds(n):
    t = 8 + n
    return t, max(0, t - 1)

def compute_commit_guards(n, acq, ctrl_tauc, ctrl_ack, special_index):
    EPS = 1
    ctrl_sum = ctrl_tauc + ctrl_ack
    C_special = acq - ctrl_sum - EPS
    if C_special < 0:
        C_special = 0

    commits = {}
    base_increase = 5
    for i in range(1, n+1):
        if i == special_index:
            commits[i] = int(C_special)
        else:
            val = int(C_special + base_increase + (i - 1) * 5)
            if val <= C_special:
                val = int(C_special + base_increase + 5)
            commits[i] = val
    return commits

def write_model(n, filename):
    if n < 2:
        raise ValueError("n must be >= 2")

    acq = acq_upper_bound(n)
    ctrl_tauc, ctrl_ack = controller_thresholds(n)
    special = n - 1
    commits = compute_commit_guards(n, acq, ctrl_tauc, ctrl_ack, special)

    parts = []

    parts.append("system:fischer")
    parts.append("")

    for i in range(1, n+1):
        parts.append(f"clock:1:x{i}")
    parts.append("")
    for i in range(1, n+1):
        parts.append(f"clock:1:y{i}")
    parts.append("")

    for i in range(1, n+1):
        parts.append(f"event:check{i}")
    parts.append("")
    for i in range(1, n+1):
        parts.append(f"event:acq{i}")
        parts.append(f"event:cs{i}")
        parts.append(f"event:rel{i}")
        parts.append("")
    for i in range(1, n+1):
        parts.append(f"event:commit{i}")
    for i in range(1, n+1):
        parts.append(f"event:tauc{i}")
    for i in range(1, n+1):
        parts.append(f"event:ack{i}")
    parts.append("")

    for i in range(1, n+1):
        parts.append(f"process:p{i}")
        parts.append(f"location:p{i}:ncs{{initial:}}")
        parts.append(f"location:p{i}:req{{}}")
        parts.append(f"location:p{i}:wait{{}}")
        parts.append(f"location:p{i}:commit{{}}")
        parts.append(f"location:p{i}:ack{{}}")
        parts.append(f"location:p{i}:cs{{}}")
        parts.append("")

        parts.append(f"edge:p{i}:ncs:req:check{i}" + "{do:x" + f"{i}=0" + "}")
        parts.append(f"edge:p{i}:req:wait:acq{i}" + "{do:x" + f"{i}=0:provided:(x{i}<={acq})" + "}")
        parts.append(f"edge:p{i}:req:req:check{i}" + "{do:x" + f"{i}=0" + "}")
        parts.append(f"edge:p{i}:wait:commit:commit{i}" + f"{{provided:(x{i}>{commits[i]})}}")
        parts.append(f"edge:p{i}:commit:ack:ack{i}" + "{}")
        parts.append(f"edge:p{i}:ack:cs:cs{i}" + "{}")
        parts.append(f"edge:p{i}:wait:req:check{i}" + "{do:x" + f"{i}=0" + "}")
        parts.append(f"edge:p{i}:cs:ncs:rel{i}" + "{}")
        parts.append("")

        parts.append(f"process:p{i}_c")
        parts.append(f"location:p{i}_c:idle{{initial:}}")
        parts.append(f"location:p{i}_c:setup{{}}")
        parts.append(f"location:p{i}_c:ready{{}}")
        parts.append(f"location:p{i}_c:authorized{{}}")
        parts.append("")

        parts.append(f"edge:p{i}_c:idle:setup:commit{i}" + "{do:y" + f"{i}=0" + "}")
        parts.append(f"edge:p{i}_c:setup:ready:tauc{i}" + "{do:y" + f"{i}=0:provided:(y{i}>{ctrl_tauc})" + "}")
        parts.append(f"edge:p{i}_c:ready:authorized:ack{i}" + f"{{provided:(y{i}>{ctrl_ack})}}")
        parts.append("")

    parts.append("process:shared_var")
    parts.append("location:shared_var:free{initial:}")
    for i in range(1, n+1):
        parts.append(f"location:shared_var:p{i}{{}}")
    parts.append("")

    for i in range(1, n+1):
        parts.append(f"edge:shared_var:free:free:check{i}" + "{}")
    parts.append("")

    for j in range(1, n+1):
        parts.append(f"edge:shared_var:free:p{j}:acq{j}" + "{}")
        for k in range(1, n+1):
            if k == j:
                continue
            parts.append(f"edge:shared_var:p{k}:p{j}:acq{j}" + "{}")
    parts.append("")

    for i in range(1, n+1):
        parts.append(f"edge:shared_var:p{i}:free:rel{i}" + "{}")
    parts.append("")

    for i in range(1, n+1):
        parts.append(f"edge:shared_var:p{i}:p{i}:cs{i}" + "{}")
    parts.append("")

    parts.append("process:prop")
    parts.append("")
    parts.append("location:prop:NoCS{initial:}")
    for i in range(1, n+1):
        parts.append(f"location:prop:P{i}_inCS{{}}")
    parts.append("location:prop:PI{labels:Pi}")
    parts.append("")

    for i in range(1, n+1):
        parts.append(f"edge:prop:P{i}_inCS:NoCS:rel{i}" + "{}")
    for i in range(1, n+1):
        parts.append(f"edge:prop:NoCS:P{i}_inCS:cs{i}" + "{}")
    parts.append("")

    for i in range(1, n+1):
        for j in range(1, n+1):
            if i == j:
                continue
            parts.append(f"edge:prop:P{i}_inCS:PI:cs{j}" + "{}")
    parts.append("")

    for i in range(1, n+1):
        parts.append(f"sync:shared_var@check{i}:p{i}@check{i}")
        parts.append(f"sync:shared_var@acq{i}:p{i}@acq{i}")
        parts.append(f"sync:shared_var@cs{i}:p{i}@cs{i}:prop@cs{i}")
        parts.append(f"sync:shared_var@rel{i}:p{i}@rel{i}:prop@rel{i}")
    parts.append("")

    for i in range(1, n+1):
        parts.append(f"sync:p{i}@commit{i}:p{i}_c@commit{i}")
    for i in range(1, n+1):
        parts.append(f"sync:p{i}@ack{i}:p{i}_c@ack{i}")
    parts.append("")

    model_text = "\n".join(parts) + "\n"

    with open(filename, "w", encoding="utf-8") as f:
        f.write(model_text)

    print(f"Wrote model for n={n} to '{filename}'")

def main(argv):
    if len(argv) != 2:
        print("Usage: python FischerI.py N")
        sys.exit(1)
    try:
        n = int(argv[1])
    except ValueError:
        print("N must be an integer.")
        sys.exit(1)
    filename = f"FischerI_{n}"
    write_model(n, filename)

if __name__ == "__main__":
    main(sys.argv)
