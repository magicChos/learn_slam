#!/usr/bin/env python
import os

nodes = os.popen("rosnode list").readlines()
for i in range(len(nodes)):
    nodes[i] = nodes[i].replace("\n", "")
    print(nodes[i])

for node in nodes:
    os.system("rosnode kill " + node)
