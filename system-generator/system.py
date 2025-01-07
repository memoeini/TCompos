"""
Part of the TCompos project. See files AUTHORS and LICENSE for copyright details.
"""

import re
import argparse

parser = argparse.ArgumentParser(description="Process automata lists.")
parser.add_argument(
    "--prop_automata",
    nargs="+",
    default=["schedule", "task1", "buffer1", "runnable1", "runnable2", "prop1"],
    help="List of prop automata",
)
parser.add_argument(
    "--env_automata",
    nargs="+",
    default=[
        "buffer2",
        "buffer3",
        "task2",
        "task3",
        "runnable3",
        "runnable4",
        "runnable5",
        "runnable6",
        "runnable7",
    ],
    help="List of env automata",
)
args = parser.parse_args()


prop_automata = args.prop_automata
env_automata = args.env_automata

print("Prop automata: ", prop_automata)
print("Env automata: ", env_automata)

# Get the input automata code (this can be from a file or string)
f = open("input", "r")

system_desc = {
    "system": None,
    "clocks": [],
    "int_vars": [],
    "events": [],
    "syncs": [],
    "processes": {},
}
clock_set = set()
for line in f.readlines():
    # if (line.startswith('\n')): continue
    comment_out_line = line.split("#")[0].strip()
    parsed_line = comment_out_line.split(":")
    lowered_line = comment_out_line.lower()
    if lowered_line.startswith("system:"):
        system_desc["system"] = parsed_line[1]
    elif lowered_line.startswith("clock:"):
        system_desc["clocks"].append(
            {
                "name": parsed_line[-1],
                "exp": comment_out_line,
            }
        )
        clock_set.add(parsed_line[-1])
    elif lowered_line.startswith("int:"):
        system_desc["int_vars"].append(comment_out_line)
    elif lowered_line.startswith("event:"):
        system_desc["events"].append(parsed_line[1])
    elif lowered_line.startswith("location:"):
        system_desc["processes"][parsed_line[1]]["locations"].append(comment_out_line)
    elif lowered_line.startswith("edge:"):
        system_desc["processes"][parsed_line[1]]["edges"].append(comment_out_line)
    elif lowered_line.startswith("process:"):
        system_desc["processes"][parsed_line[1]] = {
            "process": comment_out_line,
            "locations": [],
            "edges": [],
        }
    elif lowered_line.startswith("sync:"):
        processes = []
        for s in parsed_line[1:]:
            processes.append(s.split("@")[0].strip())
        system_desc["syncs"].append({"processes": processes, "sync": comment_out_line})


prop_process_set = set(prop_automata)
prop_event_set = set()
prop_clock_set = set()
env_process_set = set(env_automata)
env_event_set = set()
env_clock_set = set()


# extracts and tokenize different parts (operators, variables, numbers) of an expression (i.e. c += 1)
def extract_parts(expression):
    # regular expression to match different components of an exp
    pattern = r"(\w+|[+\-*/%=<>!&|^]+|\d+|\S)"

    parts = re.findall(pattern, expression.replace(" ", ""))

    return parts


# find clocks that used in processes' locations
for process in prop_process_set:
    for loc in system_desc["processes"][process]["locations"]:
        do_provided_invariant = loc.split("{")[1].split("}")[0].strip().split(":")
        if len(do_provided_invariant) > 0:
            expression = None
            for i in range(len(do_provided_invariant)):
                if do_provided_invariant[i] == "provided":
                    expression = do_provided_invariant[i + 1].strip()

                elif do_provided_invariant[i] == "do":
                    expression = do_provided_invariant[i + 1].strip()

                elif do_provided_invariant[i] == "invariant":
                    expression = do_provided_invariant[i + 1].strip()

            if expression:
                tokens = extract_parts(expression)
                for t in tokens:
                    if t in clock_set:
                        prop_clock_set.add(t)

for process in env_process_set:
    for loc in system_desc["processes"][process]["locations"]:
        do_provided_invariant = loc.split("{")[1].split("}")[0].strip().split(":")
        if len(do_provided_invariant) > 0:
            expression = None
            for i in range(len(do_provided_invariant)):
                if do_provided_invariant[i] == "provided":
                    expression = do_provided_invariant[i + 1].strip()

                elif do_provided_invariant[i] == "do":
                    expression = do_provided_invariant[i + 1].strip()

                elif do_provided_invariant[i] == "invariant":
                    expression = do_provided_invariant[i + 1].strip()

            if expression:
                tokens = extract_parts(expression)
                for t in tokens:
                    if t in clock_set:
                        env_clock_set.add(t)

# add relevant edges
for process in prop_process_set:
    for edge in system_desc["processes"][process]["edges"]:
        event = edge.split("{")[0].split(":")[4].strip()
        do_provided_invariant = edge.split("{")[1].split("}")[0].strip().split(":")
        if len(do_provided_invariant) > 0:
            expression = None
            for i in range(len(do_provided_invariant)):
                if do_provided_invariant[i] == "provided":
                    expression = do_provided_invariant[i + 1].strip()

                elif do_provided_invariant[i] == "do":
                    expression = do_provided_invariant[i + 1].strip()

                elif do_provided_invariant[i] == "invariant":
                    expression = do_provided_invariant[i + 1].strip()

            if expression:
                tokens = extract_parts(expression)
                for t in tokens:
                    if t in clock_set:
                        prop_clock_set.add(t)
        prop_event_set.add(event)

for process in env_process_set:
    for edge in system_desc["processes"][process]["edges"]:
        event = edge.split("{")[0].split(":")[4].strip()
        do_provided_invariant = edge.split("{")[1].split("}")[0].strip().split(":")
        if len(do_provided_invariant) > 0:
            expression = None
            for i in range(len(do_provided_invariant)):
                if do_provided_invariant[i] == "provided":
                    expression = do_provided_invariant[i + 1].strip()

                elif do_provided_invariant[i] == "do":
                    expression = do_provided_invariant[i + 1].strip()

                elif do_provided_invariant[i] == "invariant":
                    expression = do_provided_invariant[i + 1].strip()

            if expression:
                tokens = extract_parts(expression)
                for t in tokens:
                    if t in clock_set:
                        env_clock_set.add(t)
        env_event_set.add(event)

underlined_actions_set = prop_event_set - env_event_set
underlined_actions = list(underlined_actions_set)
prop_actions = list(prop_event_set - underlined_actions_set)
for event in underlined_actions:
    prop_actions.append("_{}".format(event))

for i in range(len(system_desc["events"])):
    if system_desc["events"][i] in underlined_actions:
        system_desc["events"][i] = "_{}".format(system_desc["events"][i])

for process in system_desc["processes"].keys():
    for i in range(len(system_desc["processes"][process]["edges"])):
        e = system_desc["processes"][process]["edges"][i]
        event = e.split(":")[4].split("{")[0]

        if event in underlined_actions:
            idx = e.find(":{}".format(event) + "{") + 1
            system_desc["processes"][process]["edges"][i] = e[:idx] + "_" + e[idx:]

for i in range(len(system_desc["syncs"])):
    sync = system_desc["syncs"][i]["sync"]
    syncs = sync.split(":")[1:]
    new_sync = "sync"
    for s in syncs:
        if s:
            parsed_s = s.split("@")
            if parsed_s[1] in underlined_actions:
                new_sync += ":{}@_{}".format(parsed_s[0], parsed_s[1])
            else:
                new_sync += ":{}@{}".format(parsed_s[0], parsed_s[1])

    system_desc["syncs"][i]["sync"] = new_sync

prop_syncs = []
env_syncs = []
for i in range(len(system_desc["syncs"])):
    sync = system_desc["syncs"][i]["sync"]
    syncs = sync.split(":")[1:]
    new_sync_prop = {"literal": "sync", "ctr": 0}
    new_sync_env = {"literal": "sync", "ctr": 0}
    for s in syncs:
        if s:
            parsed_s = s.split("@")
            if parsed_s[0] in prop_process_set:
                new_sync_prop["literal"] += ":{}@{}".format(parsed_s[0], parsed_s[1])
                new_sync_prop["ctr"] += 1
            else:
                new_sync_env["literal"] += ":{}@{}".format(parsed_s[0], parsed_s[1])
                new_sync_env["ctr"] += 1

    if new_sync_prop["ctr"] > 1:
        prop_syncs.append(new_sync_prop["literal"])

    if new_sync_env["ctr"] > 1:
        env_syncs.append(new_sync_env["literal"])

f.close()

if len(prop_clock_set.intersection(env_clock_set)) != 0:
    raise Exception(
        "Clocks shouldn't be shared between the environment and the property"
    )

# write main
orig = open(system_desc["system"].lower() + "_orig", "w")
orig.write("system:{}".format(system_desc["system"] + "\n"))
orig.write("\n")

for clk in system_desc["clocks"]:
    orig.write(clk["exp"] + "\n")
orig.write("\n")

for int_var in system_desc["int_vars"]:
    orig.write(int_var + "\n")
orig.write("\n")

for event in system_desc["events"]:
    orig.write("event:{}".format(event) + "\n")
orig.write("\n")

for process in system_desc["processes"].keys():
    orig.write("process:{}".format(process) + "\n")
    for loc in system_desc["processes"][process]["locations"]:
        orig.write(loc + "\n")
    for edge in system_desc["processes"][process]["edges"]:
        orig.write(edge + "\n")
    orig.write("\n")
orig.write("\n")

for sync in system_desc["syncs"]:
    orig.write(sync["sync"] + "\n")
orig.close()

# write prop
prop = open(system_desc["system"].lower() + "_prop", "w")
prop.write("system:{}".format(system_desc["system"] + "\n"))
prop.write("\n")

for clk in prop_clock_set:
    clock = next((c for c in system_desc["clocks"] if c["name"] == clk), None)
    if clock:
        prop.write(clock["exp"] + "\n")
prop.write("\n")

for int_var in system_desc["int_vars"]:
    prop.write(int_var + "\n")
prop.write("\n")

for event in prop_actions:
    prop.write("event:{}".format(event) + "\n")
prop.write("\n")

for process in prop_automata:
    prop.write("process:{}".format(process) + "\n")
    for loc in system_desc["processes"][process]["locations"]:
        prop.write(loc + "\n")
    for edge in system_desc["processes"][process]["edges"]:
        prop.write(edge + "\n")
    prop.write("\n")
prop.write("\n")

for sync in prop_syncs:
    prop.write(sync + "\n")
prop.close()

# write env
env = open(system_desc["system"].lower() + "_env", "w")
env.write("system:{}".format(system_desc["system"] + "\n"))
env.write("\n")

for clk in env_clock_set:
    clock = next((c for c in system_desc["clocks"] if c["name"] == clk), None)
    if clock:
        prop.write(clock["exp"] + "\n")

for int_var in system_desc["int_vars"]:
    env.write(int_var + "\n")
env.write("\n")

env_event = list(env_event_set)
for event in env_event:
    env.write("event:{}".format(event) + "\n")
env.write("\n")

for process in env_automata:
    env.write("process:{}".format(process) + "\n")
    for loc in system_desc["processes"][process]["locations"]:
        env.write(loc + "\n")
    for edge in system_desc["processes"][process]["edges"]:
        env.write(edge + "\n")
    env.write("\n")
env.write("\n")

for sync in env_syncs:
    env.write(sync + "\n")
env.close()
