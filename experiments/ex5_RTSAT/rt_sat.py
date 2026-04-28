# Usage (default) : python rt_sat.py <benchmark_class> <number>
#   benchmark_class: "sat" or "unsat"
#   number: index of the benchmark to generate (from data.py)

import argparse
from itertools import product
from io import StringIO
from data import *

class rtsat_multi_sync_single_event_fixed:
    def __init__(self, graph, bound=None, output_file=None):
        self.graph = graph
        self.bound = bound
        self.output_file = output_file

    def all_ids(self):
        gate_ids = {n[0] for n in self.graph}
        input_ids = set()
        for _, _, _, _, _, links in self.graph:
            for x in links:
                if x not in gate_ids:
                    input_ids.add(x)
        return sorted(gate_ids), sorted(input_ids), sorted(gate_ids | input_ids)

    def dump(self):
        out = StringIO()
        graph = self.graph
        gate_ids, input_ids, all_ids = self.all_ids()

        print("system:rtsat", file=out)
        print("", file=out)

        for nid, _, _, _, _, _ in graph:
            print(f"clock:1:x{nid}", file=out)
        if self.bound is not None:
            print("clock:1:t", file=out)
        print("", file=out)

        print("event:start", file=out)
        print("event:sync_done", file=out)
        print("", file=out)

        for i in all_ids:
            print(f"process:out{i}", file=out)
            print(f"location:out{i}:idle{{initial:}}", file=out)
            print(f"location:out{i}:zero{{}}", file=out)
            print(f"location:out{i}:one{{}}", file=out)
            print("", file=out)

        for nid, period, duration, _, _, _ in graph:
            print(f"process:node{nid}", file=out)
            print(f"location:node{nid}:init{{initial:}}", file=out)
            print(f"location:node{nid}:down{{}}", file=out)
            print(f"location:node{nid}:up{{}}", file=out)
            if nid == 0:
                print(f"location:node0:done{{}}", file=out)
            print("", file=out)

            print(f"edge:node{nid}:init:down:start{{}}", file=out)
            print("", file=out)

        print("process:input", file=out)
        print("location:input:init{initial:}", file=out)

        prev = "init"
        for i in input_ids:
            print(f"location:input:in{i}{{}}", file=out)
        print("location:input:done{}", file=out)
        print("", file=out)

        for i in input_ids:
            print(f"event:set_{i}", file=out)
            print(f"edge:input:{prev}:in{i}:set_{i}{{}}", file=out)
            print(f"edge:out{i}:idle:zero:set_{i}{{}}", file=out)
            print(f"edge:out{i}:idle:one:set_{i}{{}}", file=out)
            print(f"sync:input@set_{i}:out{i}@set_{i}", file=out)
            print("", file=out)

            prev = f"in{i}"
            
        print(f"edge:input:{prev}:done:start{{}}", file=out)

        for i in input_ids:
            print(f"edge:out{i}:zero:zero:start{{}}", file=out)
            print(f"edge:out{i}:one:one:start{{}}", file=out)

        start_parts = [f"input@start"] + [f"node{nid}@start" for nid, _, _, _, _, _ in graph] + [f"out{i}@start" for i in input_ids]
        print("sync:" + ":".join(start_parts), file=out)
        print("", file=out)

        for nid, period, _, _, _, links in graph:
            for lid in links:
                print(f"event:stall_{nid}_{lid}", file=out)

                print(
                    f"edge:node{nid}:down:down:stall_{nid}_{lid}"
                    f"{{provided:(x{nid}=={period}):do:x{nid}=0}}",
                    file=out
                )

                print(f"edge:out{lid}:idle:idle:stall_{nid}_{lid}{{}}", file=out)
                print("", file=out)

        sync_index = 0
        for nid, period, duration, neg, th, links in graph:
            k = len(links)

            for values in product([0, 1], repeat=k):
                s = sum(values)
                ok = (s >= th)
                if neg:
                    ok = not ok
                out_val = 1 if ok else 0

                ev = f"sync_{nid}_{sync_index}"
                sync_index += 1

                print(f"event:{ev}", file=out)

                print(
                    f"edge:node{nid}:down:up:{ev}"
                    f"{{provided:(x{nid}=={period}):do:x{nid}=0}}",
                    file=out
                )

                for lid, v in zip(links, values):
                    loc = "one" if v == 1 else "zero"
                    print(
                        f"edge:out{lid}:{loc}:{loc}:{ev}{{}}",
                        file=out
                    )

                tgt = "one" if out_val == 1 else "zero"
                print(
                    f"edge:out{nid}:idle:{tgt}:{ev}{{}}",
                    file=out
                )

                parts = [f"node{nid}@{ev}"]
                for lid in links:
                    parts.append(f"out{lid}@{ev}")
                parts.append(f"out{nid}@{ev}")
                print("sync:" + ":".join(parts), file=out)
                print("", file=out)

        for nid, _, duration, _, _, _ in graph:
            print(f"event:reset_{nid}", file=out)

            print(
                f"edge:node{nid}:up:down:reset_{nid}"
                f"{{provided:(x{nid}<={duration}):do:x{nid}=0}}",
                file=out
            )

            print(f"edge:out{nid}:zero:idle:reset_{nid}{{}}", file=out)
            print(f"edge:out{nid}:one:idle:reset_{nid}{{}}", file=out)

            print(f"sync:node{nid}@reset_{nid}:out{nid}@reset_{nid}", file=out)
            print("", file=out)

        print("process:prop", file=out)
        print("location:prop:p0{initial:}", file=out)
        print("location:prop:pi{labels:Pi}", file=out)
        print("", file=out)

        if 0 in gate_ids:
            guard = ""
            if self.bound is not None:
                guard = f"provided:(t<={self.bound})"

            print(f"edge:node0:up:done:sync_done{{{guard}}}", file=out)
            print(f"edge:prop:p0:pi:sync_done{{}}", file=out)
            print("sync:node0@sync_done:prop@sync_done", file=out)
            print("", file=out)

        with open(self.output_file, "w") as f:
            f.write(out.getvalue())


def main():
    parser = argparse.ArgumentParser(
        description="rtsat script"
    )
    parser.add_argument("benchmark_class", choices=["sat", "unsat"])
    parser.add_argument("number", type=int)
    parser.add_argument("-b", "--bound", type=int)
    args = parser.parse_args()

    benchmarks = sat_benchmarks if args.benchmark_class == "sat" else unsat_benchmarks

    if args.number not in benchmarks:
        raise Exception("benchmark index out of range")

    output_file = f"{args.benchmark_class}_{args.number}"

    rtsat_multi_sync_single_event_fixed(
        benchmarks[args.number],
        bound=args.bound,
        output_file=output_file
    ).dump()

    print(f"model written to {output_file}")


if __name__ == "__main__":
    main()
