#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2018-2020 German Aerospace Center (DLR) and others.
# See LICENSE file which accompanies this distributionf or the usage terms of these tools.

# @file    ocit2AGLOSA.py
# @author  Jakob Erdmann
# @author  Daniel Wesemeyer
# @date    2018-09-19
# @version $Id$

"""
Create configuration file for AGLOSA from ocit xml
depends on ocit2SUMO 
"""
from collections import defaultdict

import os,sys
import ocit2SUMO
from ocit2SUMO import buildSumoPhasesFromOcit


def main(options):
    with open(options.outfile, 'w') as outf:

        for nodeID, sgIndex, maxIndex, phaseProperties, comments in buildSumoPhasesFromOcit(options):
            outf.write("PHASE_PROPERTIES = { # Teilknoten %s\n" % nodeID)
            for i, ((dur, state, nextPhase, major, majorNext), comment) in enumerate(zip(phaseProperties, comments)):
                if i % 40 == 0:
                    # write index annotation
                    indexComment = ''
                    indexComment2 = ''
                    for i2 in range(len(state)):
                        indexComment += str(i2 / 10)
                        indexComment2 += str(i2 % 10)
                    stateLength = len(state)
                    outf.write('         #%s\n' % indexComment)
                    outf.write('         #%s\n' % indexComment2)


                comment = comment.replace("PÜ", "PT")

                if nextPhase == -1:
                    comment = comment[9:]
                outf.write('  %3i : ("%s", %2i, %2i, %2i), # %s\n' % (
                    i, state, dur, major, majorNext, comment))
            outf.write("}\n\n\n")




if __name__ == "__main__":
    options = ocit2SUMO.get_options()
    if options.outfile == "lsa.add.xml":
        options.outfile = "aglosacfg.py"
    if not main(options):
        sys.exit(1)
