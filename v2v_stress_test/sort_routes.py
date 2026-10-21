import xml.etree.ElementTree as ET

INPUT = "routes.rou.xml"
OUTPUT = "routes_sorted.rou.xml"

tree = ET.parse(INPUT)
root = tree.getroot()

# Elements that define objects/routes and must stay before
# timed vehicle/flow entries.
definitions = []

# Timed traffic entries: vehicle + flow.
timed_entries = []

for elem in list(root):

    if elem.tag in ("vehicle", "flow"):
        if elem.tag == "vehicle":
            time_value = float(elem.get("depart"))
        else:
            time_value = float(elem.get("begin"))

        timed_entries.append((time_value, elem))

    else:
        definitions.append(elem)

# Sort ALL timed entries together.
timed_entries.sort(key=lambda x: x[0])

# Rebuild XML:
# definitions first, then all vehicles/flows ordered by time.
root[:] = definitions + [elem for _, elem in timed_entries]

tree.write(
    OUTPUT,
    encoding="utf-8",
    xml_declaration=True
)

print(f"Created: {OUTPUT}")
print(f"Timed entries sorted: {len(timed_entries)}")

print("\nFirst timed entries:")
for t, elem in timed_entries[:10]:
    print(f"{t:6.1f}  {elem.tag:7s}  {elem.get('id')}")

print("\nLast timed entries:")
for t, elem in timed_entries[-10:]:
    print(f"{t:6.1f}  {elem.tag:7s}  {elem.get('id')}")
