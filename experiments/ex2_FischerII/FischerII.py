# Run the script with an integer argument n to generate a model with n processes

import sys

def generate_model(n: int) -> str:
    if n < 1:
        raise ValueError("n must be >= 1")

    lines = []

    lines.append("system:fischer_cl")
    lines.append("")

    for i in range(1, n + 1):
        lines.append(f"clock:1:x{i}")
    for i in range(1, n + 1):
        lines.append(f"clock:1:r{i}")
    lines.append("")

    for i in range(1, n + 1):
        lines.append(f"event:check{i}")
    lines.append("")

    lines.append("event:acq1")
    lines.append("event:cs1")
    lines.append("event:rel1")
    lines.append("event:crash_1")
    lines.append("event:log_event_1")
    lines.append("event:log_commit_1")
    lines.append("event:flush_start_1")
    lines.append("event:flush_completed_1")
    lines.append("")

    for i in range(2, n + 1):
        lines.append(f"event:acq{i}")
        lines.append(f"event:cs{i}")
        lines.append(f"event:rel{i}")
        lines.append(f"event:tau{i}")
    lines.append("")

    lines.append("event:abort_logging_1")
    lines.append("")

    for i in range(1, n + 1):
        lines.append(f"process:p{i}")
        lines.append(f"location:p{i}:ncs{{initial:}}")
        lines.append(f"location:p{i}:req{{}}")
        lines.append(f"location:p{i}:wait{{}}")
        lines.append(f"location:p{i}:cs{{}}")
        lines.append(f"location:p{i}:crashed{{}}")
        lines.append(f"location:p{i}:logging{{}}")
        lines.append(f"location:p{i}:finalizing{{}}")
        lines.append(f"location:p{i}:halted{{}}")
        lines.append("")

        lines.append(f"edge:p{i}:ncs:req:check{i}{{do:x{i}=0}}")
        lines.append(f"edge:p{i}:req:wait:acq{i}{{do:x{i}=0:provided:(x{i}<=10)}}")
        lines.append(f"edge:p{i}:req:req:check{i}{{do:x{i}=0}}")
        lines.append(f"edge:p{i}:wait:cs:cs{i}{{provided:(x{i}>10)}}")
        lines.append(f"edge:p{i}:wait:req:check{i}{{do:x{i}=0}}")
        lines.append(f"edge:p{i}:cs:ncs:rel{i}{{}}")
        lines.append("")

    lines.append("edge:p1:req:crashed:crash_1{}")
    lines.append("edge:p1:crashed:logging:crash_1{provided:(x1>5)}")
    lines.append("edge:p1:logging:finalizing:log_event_1{}")
    lines.append("edge:p1:finalizing:halted:log_commit_1{}")
    lines.append("edge:p1:halted:wait:crash_1{do:x1=0:provided:(x1<=10)}")
    lines.append("")

    lines.append("process:shared_var")
    lines.append("location:shared_var:free{initial:}")
    for i in range(1, n + 1):
        lines.append(f"location:shared_var:p{i}{{}}")
    lines.append("")

    for i in range(1, n + 1):
        lines.append(f"edge:shared_var:free:free:check{i}{{}}")
    lines.append("")

    for i in range(1, n + 1):
        lines.append(f"edge:shared_var:free:p{i}:acq{i}{{}}")
        for j in range(1, n + 1):
            if j != i:
                lines.append(f"edge:shared_var:p{j}:p{i}:acq{i}{{}}")
        lines.append("")

    for i in range(1, n + 1):
        lines.append(f"edge:shared_var:p{i}:free:rel{i}{{}}")
    lines.append("")

    for i in range(1, n + 1):
        lines.append(f"edge:shared_var:p{i}:p{i}:cs{i}{{}}")
    lines.append("")

    lines.append("process:logger1")
    lines.append("location:logger1:standby{initial:}")
    lines.append("location:logger1:collecting{}")
    lines.append("location:logger1:flushing{}")
    lines.append("location:logger1:completed{}")
    lines.append("location:logger1:aborted{}")
    lines.append("")
    lines.append("edge:logger1:standby:collecting:log_event_1{do:r1=0}")
    lines.append("edge:logger1:collecting:flushing:flush_start_1{do:r1=0:provided:(r1>2)}")
    lines.append("edge:logger1:flushing:completed:flush_completed_1{do:r1=0:provided:(r1>3)}")
    lines.append("edge:logger1:completed:standby:log_commit_1{provided:(r1==0)}")
    lines.append("edge:logger1:collecting:aborted:abort_logging_1{}")
    lines.append("")

    lines.append("process:prop")
    lines.append("")
    lines.append("location:prop:NoCS{initial:}")
    lines.append("location:prop:L_only{}")
    lines.append("location:prop:F_only{}")
    lines.append("location:prop:PI{labels:Pi}")
    lines.append("")
    lines.append("edge:prop:NoCS:L_only:cs1{}")
    lines.append("edge:prop:F_only:PI:cs1{}")
    lines.append("edge:prop:L_only:NoCS:rel1{}")
    lines.append("")

    if n >= 2:
        for i in range(2, n + 1):
            lines.append(f"edge:prop:NoCS:F_only:cs{i}{{}}")
        lines.append("")
        for i in range(2, n + 1):
            lines.append(f"edge:prop:L_only:PI:cs{i}{{}}")
        lines.append("")
        for i in range(2, n + 1):
            lines.append(f"edge:prop:F_only:NoCS:rel{i}{{}}")
        lines.append("")

    for i in range(1, n + 1):
        lines.append(f"sync:shared_var@check{i}:p{i}@check{i}")
        lines.append(f"sync:shared_var@acq{i}:p{i}@acq{i}")
        lines.append(f"sync:shared_var@cs{i}:p{i}@cs{i}:prop@cs{i}")
        lines.append(f"sync:shared_var@rel{i}:p{i}@rel{i}:prop@rel{i}")
        lines.append("")

    lines.append("sync:p1@log_event_1:logger1@log_event_1")
    lines.append("sync:p1@log_commit_1:logger1@log_commit_1")
    lines.append("")

    return "\n".join(lines)


def main():
    if len(sys.argv) != 2:
        print("Usage: python FischerII.py <n>")
        sys.exit(1)

    try:
        n = int(sys.argv[1])
    except ValueError:
        print("n must be an integer >= 1")
        sys.exit(1)

    if n < 1:
        print("n must be >= 1")
        sys.exit(1)

    model_text = generate_model(n)
    filename = f"FischerII_{n}"
    with open(filename, "w", encoding="utf-8") as f:
        f.write(model_text)

    print(f"Model written to {filename}")


if __name__ == "__main__":
    main()
