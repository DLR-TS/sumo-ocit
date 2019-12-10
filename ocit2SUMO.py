#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2018-2020 German Aerospace Center (DLR) and others.
# See LICENSE file which accompanies this distributionf or the usage terms of these tools.

# @file    ocit2SUMO.py
# @author  Jakob Erdmann
# @author  Daniel Wesemeyer
# @date    2018-09-19

"""
Create 'sumo additional file' with tls programs from ocit xml
SUMO tls indices must exist as comment (Bemerkung) in each signal group (Signalgruppe)
"""
from __future__ import absolute_import
from __future__ import print_function
import os, sys
from collections import defaultdict
from itertools import groupby
import optparse
import re
from copy import deepcopy

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import sumolib  # noqa
from sumolib.xml import parse


INDEX_SEPARATOR = ';'
SIGNALGROUPS = "SignalgruppeListe"
SIGNALGROUP = "Signalgruppe"
SHORTID = "BezeichnungKurz"
PARTIAL_NODE = "AbschaltTeilknoten"
COMMENTS = "Bemerkungen"
COMMENT = "Bemerkung"
PHASELIST = "PhaseListe"
PHASE = "Phase"
PHASE_ELEMENT = "PhasenElementeintrag"
PHASE_SIGNAL = "Signalbild"
SUCCESSOR_LIST = "PhasenfolgeListe"
SUCCESSOR = "Phasenfolge"
SUCCESSOR_ENTRY = "Uebergang"
TRANSITION_LIST = "PhasenuebergangListe"
TRANSITION = "Phasenuebergang"
SWITCHING = "Schaltinformation"
SWITCHING_DURATION = "Dauer"
SWITCHING_ELEMENT = "PUeElement"
SWITCHING_INITIAL = "StartSignalbild"
SWITCHING_TIME = "Schaltzeit"
SWITCHING_TIME2 = "Schaltzeitpunkt"
SWITCH_FROM = "VonPhase"
SWITCH_TO = "NachPhase"
PROGRAMLIST = "SignalprogrammListe"
PROGRAM = "Signalprogramm"
PROGRAM_HEAD = "SPKopfzeile"
CYCLE_TIME = "TU"
PROGRAM_ROW = "SPZeile"
PERM_STATE = "DauerSignalbild"

SIGNAL_ACTIVATE="AnwurfUebergang"
SIGNAL_ELEMENT="Uebergangselement"
SIGNAL_DURATION="Zeitdauer"
SIGNAL_DEACTIVATE="AbwurfUebergang"




SIGNALSTATE_SUMOSTATE = {
        'gruen': 'G',
        'gelb': 'y',
        'rot': 'r',
        'rotgelb': 'u',
        'dunkel': 'O',
        'gelbblk': 'o',
        '00' : 'o',
        '08' : 'o',
        '03' : 'r',
        '30' : 'G',
        '0F' : 'u',
        '0C' : 'y'
        }


SUMOSTATE_COMPLEX = {
        'rO'  : 'r',
        'Or'  : 'r',
        'ro'  : 'r',
        'rG'  : 'r',
        'Go'  : 'g',
        'go'  : 'g',
        'gO'  : 'g',
        'OG'  : 'G',
        'Gr'  : 'G',
        'GO'  : 'G',
        'yG'  : 'y',
        'yo'  : 'y',
        'yO'  : 'y',
        'yr'  : 'r',
        'uG'  : 'u',
        'uo'  : 'u',
        'uO'  : 'u',
        'ur'  : 'r',

        'rro' : 'r',
        'rrO' : 'r',
        'rOo' : 'r',
        'rGo' : 'r',
        'rGO' : 'r',
        'rOO' : 'r',
        'roO' : 'r',
        'rOG' : 'r',
        'OOr' : 'r',
        'OGo' : 'g',
        'OGO' : 'G',
        'OrO' : 'r',
        'gOo' : 'g',
        'gGO' : 'G',
        'Goo' : 'G',
        'GoO' : 'g',
        'GOo' : 'g',
        'GOO' : 'g',
        'goO' : 'g',
        'GoG' : 'G',
        'GOG' : 'G',
        'GGO' : 'G',
        'GGo' : 'g',
        'GrO' : 'G',
        'rGG' : 'g',
        'uGO' : 'G',
        'uoO' : 'u',
        'uGG' : 'u',
        'yOG' : 'y',
        'yGo' : 'y',
        'yGO' : 'y',
        'yGG' : 'y',
        'Ogo' : 'g',
        'OgO' : 'g',
        'Oro' : 'r',
        'Orr' : 'r',
        'OGr' : 'G',
        'Grr' : 'G',
        'rgO' : 'r',
        'rgo' : 'r',
        'Gro' : 'G',
        'OrG' : 'r',
        'uOG'  : 'u',
        }
def interpret_complex_state(state, phaseID, index, index2groups):
    # check whether all letters are the same
    if len(state) * state[0] == state:
        return state[0]

    singleLetter = SUMOSTATE_COMPLEX.get(state, state)
    if len(singleLetter) > 1:
        sys.stderr.write("Invalid singleLetter '%s' for index %s in phase '%s' (state '%s' groups=%s)\n" % (
            singleLetter, index, phaseID, state, index2groups[index]))
        assert(False)

    return singleLetter

def normalizeStates(states, phaseID, index2groups, minorIndex, majorIndex=[]):
    for i, state in enumerate(states):
        for index in index2groups[i]:
            if index in majorIndex:
                states[i] = 'G'
                continue
        states[i] = interpret_complex_state(''.join(state), phaseID, i, index2groups)
        if states[i] == 'G' and i in minorIndex:
            states[i] = 'g'
    return states

def isBlinker(groupID):
    return "BL" in groupID or "ge" in groupID or "gn" in groupID or "H" in groupID

# regex for finding consecutive digits in a string
RENUM = re.compile("(\d\d*)")

def get_options(args=None):
    optParser = optparse.OptionParser()
    optParser.add_option("--tls-id", dest="tlsID", default="TLS_ID",
                         help="declare the traffic light ID used in the SUMO network")
    optParser.add_option("-o", "--output-file", dest="outfile",
                         default="lsa.add.xml", help="define the output filename")
    optParser.add_option("--min-duration", dest="defaultMinDur", type="int",
                         default=5, help="default minimum duration for main phases")
    optParser.add_option("--phase-duration", dest="phaseDuration", type="int",
                         default=100, help="default duration for main phases")
    optParser.add_option("--minor-index", dest="minorIndex", help="The given list of indices always gets minor green")
    optParser.add_option("--major-index", dest="majorIndex", help="The given list of signal groups always gets major green. When the respective signal is switched on and they are combined with other signals, it overrides the other signals' information.")
    optParser.add_option("--ignore-phases", dest="ignorePhases", help="ignore the comma-separated list of phase indices")
    optParser.add_option("--ignore-nodes", dest="ignoreNodes", help="ignore the comma-separated list of node indices")
    optParser.add_option("--no-grouping", action="store_true", default=False, dest="noGrouping", help="Equal phases are not grouped in the end, i.e. the tlLogic consists of n phases (n = cycle time) with length of 1 sec. each.")
    optParser.add_option("--use-programs", action="store_true", default=False, dest="usePrograms",
                        help="Determines whether the signal programs that are defined in the OCIT should be used to generate cycles or not")
    optParser.add_option("-v", "--verbose", action="store_true",
                         default=False, help="tell me what you are doing")
    optParser.add_option("--verbose-index", type='int', dest="vIndex", help="Give extra information on the given link index")
    optParser.add_option("--verbose-group", dest="vGroup", help="Give extra information on the given signal gropu")
    optParser.add_option("--verbose-transition", dest="vTrans", help="Give extra information on the transition")

    (options, args) = optParser.parse_args(args=args)
    
    options.ignorePhases = set(map(int, options.ignorePhases.split(','))) if options.ignorePhases else set()
    options.ignoreNodes = set(map(int, options.ignoreNodes.split(','))) if options.ignoreNodes else set()
    options.minorIndex = set(map(int, options.minorIndex.split(','))) if options.minorIndex else set()

    if len(args) == 1:
        options.ocitfile = args[0]
    else:
        sys.stderr.write("argument <OCITFILE> missing")
        sys.exit()
        
    return options

def extraInfo(options, trans, group, index=None):
    return ((options.vIndex or options.vTrans or options.vGroup) 
            and (options.vIndex is None or index is None or options.vIndex == index)
            and (options.vTrans is None or trans is None or options.vTrans in trans)
            and (options.vGroup is None or group is None or options.vGroup == group))


def textNode(parent, childAsString):
    """return the text value from a single child element with the given name"""
    return parent.getChild(childAsString)[0].getText()

def getSignalDuration(sg, child, indices, into, ID):
    try:
        el1 = sg.getChild(child)[0]
        el2 = el1.getChild(SIGNAL_ELEMENT)[0]
        for i in indices:
            into[i] = max(into[i], int(textNode(el2, SIGNAL_DURATION)))
        # just in case we have a directional signal and a regular signal controlling one and the same connection
        # we need to know that the directional signal does not have red yellow time
        into[ID] = max(into[ID], int(textNode(el2, SIGNAL_DURATION)))
    except:
        pass

def parsePhaseIDs(ocitfile):
    phaseList = list(parse(ocitfile, PHASELIST))[0]
    return sorted([textNode(p, SHORTID) for p in phaseList.getChild(PHASE)])


def parseSignalGroups(ocitfile, nodeIndex):
    partialNodeSG = defaultdict(list)
    sgIndex = defaultdict(list) # signal group id -> [linkIndex1, linkIndex2, ...]
    yellowDur = defaultdict(lambda : 0) # link index -> duration
    redYellowDur = defaultdict(lambda : 0) # link index -> duration
    maxIndex = -1
    sgList = list(parse(ocitfile, SIGNALGROUPS))[0]

    for sg in sgList.getChild(SIGNALGROUP):
        ID = textNode(sg, SHORTID)
        nodeIndex = int(textNode(sg, PARTIAL_NODE)) if sg.hasChild(PARTIAL_NODE) else nodeIndex
        #if nodeIndex is not None:
        #    # only return sigal groups that belong to the current node
        #    # ('Teilknoten'). This is done by matching with the numerical index of
        #    # the first phase in the current cycle 
        #    numID = int(RENUM.search(ID).groups()[0])
        #    # node 1 (phases 11 to 19) uses signal groups 0-9
        #    # node 2 (phases 21 to 29) uses signal groups 10-19
        #    # ...
        #    if numID < 10 * (nodeIndex - 1) or numID >= nodeIndex * 10:
        #        continue

        try:
            comments = sg.getChild(COMMENTS)[0]
            comment = textNode(comments, COMMENT)
            if comment is not None:
                indices = list(map(int, comment.split(INDEX_SEPARATOR)))
                sgIndex[ID] = indices
                maxIndex = max(maxIndex, *indices)
                # read yellow / redyellow duration (not for blinkers)
                if not isBlinker(ID):
                    getSignalDuration(sg, SIGNAL_ACTIVATE, indices, redYellowDur, ID)
                    getSignalDuration(sg, SIGNAL_DEACTIVATE, indices, yellowDur, ID)
            partialNodeSG[nodeIndex].append(ID)
        except:
            pass
    return sgIndex, yellowDur, redYellowDur, maxIndex, partialNodeSG

def getGroupPriority(groupID):
    if isBlinker(groupID):
        return 2
    if 'R' in groupID or 'L' in groupID:
        return 0
    else: 
        return 1

def getIndex2Groups(sgIndex):
    index2groups = defaultdict(list)
    for group in sorted(sgIndex.keys(), key=getGroupPriority):
        for i in sgIndex[group]:
            index2groups[i].append(group)
    return index2groups

def parsePhaseStates(ocitfile, cycle, sgIndex, maxIndex):
    index2groups = getIndex2Groups(sgIndex)
    defaultState = [['O'] * max(1, len(index2groups[i])) for i in range(maxIndex + 1)]
    #print(sgIndex)
    #print(index2groups)
    #print(defaultState)
    phases = {} # id -> sumoState
    phaseList = list(parse(ocitfile, PHASELIST))[0]
    for phase in phaseList.getChild(PHASE):
        ID = textNode(phase, SHORTID)
        if not ID in cycle:
            continue
        stateList = deepcopy(defaultState)
        for sg in phase.getChild(PHASE_ELEMENT):
            groupID = textNode(sg, SIGNALGROUP)
            groupState = SIGNALSTATE_SUMOSTATE[textNode(sg, PHASE_SIGNAL)]
            for index in sgIndex[groupID]:
                gIndex = index2groups[index].index(groupID)
                stateList[index][gIndex] = groupState
        phases[ID] = stateList
        #print(ID, stateList)
    return phases


def reduceComplexState(options, index2groups, sgIndex, ID, phases, fromPhase):
    initialState = []
    for index, complexState in enumerate(phases[fromPhase]):
        # interpret non-blinker states
        complexState2 = []
        for gIndex, groupID in enumerate(index2groups[index]):
            gState = complexState[gIndex]
            if isBlinker(groupID):
                complexState2.append(gState)
            else:
                singleLetter = interpret_complex_state(''.join(complexState), ID, index, index2groups)
                assert(len(singleLetter) == 1)
                if gState == 'G' and singleLetter == 'g':
                    complexState2.append(singleLetter)
                else:
                    complexState2.append(gState)
                    
        if not complexState2:
            complexState2 = ['O']
        if extraInfo(options, ID, None, index):
            print('\n%s' % ID, "i=%s" % index, "default", complexState2)

        initialState.append(complexState2)
    return initialState



def parseTransitions(options, sgIndex, phases):
    transitions = {} # id -> [sumoState1, sumoState2, ...]
    transitionList = list(parse(options.ocitfile, TRANSITION_LIST))[0]
    index2groups = getIndex2Groups(sgIndex)

    for transition in transitionList.getChild(TRANSITION):
        ID = textNode(transition, SHORTID)
        switching = transition.getChild(SWITCHING)[0]
        duration = max(1, int(textNode(switching, SWITCHING_DURATION)))
        if not transition.hasChild(SWITCH_FROM):
            # if no from phase was given, we cannot add a transition
            continue
        fromPhase = textNode(transition, SWITCH_FROM)
        toPhase = textNode(transition, SWITCH_TO)
        if not fromPhase in phases or not toPhase in phases:
            continue

        initialState = reduceComplexState(options, index2groups, sgIndex, ID, phases, fromPhase)

        transitionStates = [deepcopy(initialState) for t in range(duration)]

        for sg in switching.getChild(SWITCHING_ELEMENT):
            groupID = textNode(sg, SIGNALGROUP)
            if sg.hasChild(SWITCHING_INITIAL):
                initial = SIGNALSTATE_SUMOSTATE[textNode(sg, SWITCHING_INITIAL)]
            else:
                signalIndex = sgIndex[groupID]
                if len(signalIndex) > 0:
                    initial = phases[fromPhase][signalIndex[0]][0]
            if extraInfo(options, ID, groupID):
                print(ID, groupID, "init", initial, textNode(sg, SWITCHING_INITIAL), sgIndex[groupID])
            for time in range(duration):
                for index in sgIndex[groupID]:
                    gIndex = index2groups[index].index(groupID)
                    transitionStates[time][index][gIndex] = initial
            for switchObject in sg.getChild(SWITCHING_TIME):
                switchTime = int(textNode(switchObject, SWITCHING_TIME2))
                groupState = SIGNALSTATE_SUMOSTATE[textNode(switchObject, PHASE_SIGNAL)]
                if extraInfo(options, ID, groupID):
                    print(ID, groupID, "t=%s" % switchTime, groupState, textNode(switchObject, PHASE_SIGNAL), sgIndex[groupID])
                for time in range(switchTime, duration):
                    for index in sgIndex[groupID]:
                        gIndex = index2groups[index].index(groupID)
                        transitionStates[time][index][gIndex] = groupState
        transitions[(ID, fromPhase, toPhase)] = transitionStates
        if extraInfo(options, ID, None):
            print(ID)
            print("\n".join(map(str,transitionStates)))
    
    return transitions

def recheckGreenMinor(options, phases, transitions):
    # ensure that indices that start with 'g' and do not end with 'G' do not have 'G' in their transition
    for (transitionID, fromPhase, toPhase), transition in transitions.items():
        beforeStates = phases[fromPhase]
        afterStates = phases[toPhase]
        for index, (before, after) in enumerate(zip(beforeStates, afterStates)):
            if before == 'g' and after != 'G':
                #verbose = options.verbose or extraInfo(options, transitionID, None, index)
                #if verbose:
                #    print("check transition consistency for %s, index=%s" % (transitionID, index))
                corrected = []
                for t, state in enumerate(transition):
                    if state[index] == 'G':
                        state[index] = 'g'
                        corrected.append(t)
                if corrected:
                    print("corrected inconsisted state in transition '%s' index %s times %s" % (
                        transitionID, index, corrected))
    

def addRedYellow(transitions, phases, yellowDur, redYellowDur, verbose):
    for (ID, fromPhase, toPhase), transition in transitions.items():
        statesBefore = phases[fromPhase]
        statesAfter = phases[toPhase]
        for i in range(0, len(statesAfter)):
            stateBefore = statesBefore[i]
            for t in range(0, len(transition)):
                newState = transition[t][i]
                if newState == 'r' and stateBefore in ['g', 'G']:
                    for t2 in range(t, t + yellowDur[i]):
                        transition[t2][i] = 'y'
                    break;
                stateBefore = newState

            stateAfter = statesAfter[i]
            for t in range(len(transition) - 1, -2 , -1):
                if t == -1: 
                    oldState = statesBefore[i]
                else:
                    oldState = transition[t][i]
                #if fromPhase == "Phase 1" and toPhase == "Phase 6":
                #    print(i, oldState, stateAfter)
                if oldState == 'r' and stateAfter in ['g', 'G']:
                    if redYellowDur[i] == 1:
                        try:
                            transition[t + 1][i] = 'u'
                        except:
                            pass
                        break;
                stateAfter = oldState

        if verbose:
            print(ID)
            print(" ", list(statesBefore), "(before)")
            print("\n".join(["%i %s" % (i, t) for i, t in enumerate(transition)]))
            print(" ", list(statesAfter), "(after)")


def compactifyTransitions(transitions):
    timedTransitions = {} # id -> [(duration, sumoState1(, (duration2, sumoState2), ...]
    for (transitionID, fromPhase, toPhase), transition in transitions.items():
        fromTo = (fromPhase, toPhase)
        grouped = [(k, list(g)) for k,g  in groupby(transition)]
        timedTransition = [(len(g), k) for k, g in grouped]
        timedTransitions[(transitionID, fromTo)] = timedTransition
    return timedTransitions


def generateSumoPhases(cycle, majorIndex, defaultMinDur, sgIndex, phases,
        timedTransitions):
    phaseProperties = [] # [(dur, state, next, major, majorNext), ....]
    comments = [] # sumoindex, phaseID
    usedTransitions = set()
    sumoIndex = {} # phaseID -> sumo phase index
    numPhases = 0
    for phaseIndex, phaseID in enumerate(cycle):
        phaseProperties.append((defaultMinDur, phases[phaseID], -1, majorIndex[phaseID], 0))
        comments.append('          <!-- %3s %s -->' % (numPhases, phaseID))
        sumoIndex[phaseID] = numPhases
        numPhases += 1
        fromTo = (phaseID, cycle[(phaseIndex + 1) % len(cycle)])
        if fromTo in timedTransitions:
            transitionID, timedTransition = timedTransitions[fromTo]
            usedTransitions.add(fromTo)
            numPhases = appendTransition(phaseProperties, comments, timedTransition, transitionID,
                    sumoIndex, majorIndex, fromTo, numPhases)
    for (transitionID, fromTo), timedTransition in timedTransitions.items():
        if fromTo in usedTransitions:
            continue
        numPhases = appendTransition(phaseProperties, comments, timedTransition, transitionID,
                sumoIndex, majorIndex, fromTo, numPhases)

    return phaseProperties, comments


def appendTransition(phaseProperties, comments, timedTransition, transitionID,
        sumoIndex, majorIndex, fromTo, numPhases):
    fromPhase, toPhase = fromTo
    for i, (dur, state) in enumerate(timedTransition):
        try:
            nextPhase = sumoIndex[toPhase] if i == len(timedTransition) - 1 else -1
        except:
            nextPhase = -1
        nextPadding = '          ' if nextPhase == -1 else ' '
        phaseProperties.append((dur, ''.join(state), nextPhase, majorIndex[fromPhase], majorIndex[toPhase]))
        comments.append('%s<!-- %3s %s -->' % (nextPadding, numPhases, transitionID))
        numPhases += 1
    return numPhases


def writeTLS(outf, tlsID, phaseDuration, sgIndex, maxIndex, phaseProperties, comments):
    phaseDurLen = len(str(phaseDuration))
    outf.write("   <!-- sumo index to signal groups\n")
    for i in range(0, maxIndex + 1):
        sgs = []
        for sg, indices in sgIndex.items():
            for i2 in indices:
                if i2 == i:
                    sgs.append(sg)
        outf.write("   %2s: %s\n" % (i, ' '.join(sgs) ))

    outf.write("   -->\n")
    outf.write('   <tlLogic id="%s" programID="ocit_import">\n' % tlsID)
    for (dur, state, nextPhase, major, majorNext), comment in zip(phaseProperties, comments):
        if nextPhase >= 0:
            nextPhaseStr = ' next="%s"' % nextPhase
        else:
            nextPhaseStr = ''
        if majorNext == 0:
            dur = phaseDuration
        durationPadding = (phaseDurLen - len(str(dur))) * ' '
        outf.write('       <phase duration="%s"%s state="%s"%s/>%s\n' % (
            dur, durationPadding, state, nextPhaseStr, comment))
    outf.write('   </tlLogic>\n')

def buildCycles(phases, ignorePhases):
    #majorIndex = dict([(p, i + 1) for i, p in enumerate(cycle)])
    majorIndex = dict([(p, int(RENUM.search(p).groups()[0])) for p in phases])
    cycle = []
    for p in phases:
        if majorIndex[p] in ignorePhases:
            continue
        if majorIndex[p] % 10 == 1:
            if cycle:
                yield cycle, majorIndex
                cycle = []
        cycle.append(p)
    yield cycle, majorIndex


def parseSignalPrograms(options, sgIndex, redYellowDur, yellowDur):
    programs = {}
    maxSgIndex = max([val for l in sgIndex.values() for val in l])
    programsList = list(parse(options.ocitfile, PROGRAMLIST))[0]
    for program in programsList.getChild(PROGRAM):
        programID = textNode(program, SHORTID)

        import pdb

        cycleTime = int(textNode(program.getChild(PROGRAM_HEAD)[0], CYCLE_TIME))
        # generate an allread program as starting point
        sigProgram = [[[] for _ in range(maxSgIndex+1)] for _ in range(cycleTime)]
        if program.hasChild(PROGRAM_ROW):
            for element in program.getChild(PROGRAM_ROW):
                sgID = textNode(element, SIGNALGROUP)
                if not sgID in sgIndex:
                    continue
                switches = []
                permState = None
                stdClosed = 'r'
                if element.hasChild(SWITCHING_TIME):
                    for switch in element.getChild(SWITCHING_TIME):
                        state = SIGNALSTATE_SUMOSTATE[textNode(switch, PHASE_SIGNAL)]
                        t = int(textNode(switch, SWITCHING_TIME2))
                        switches.append((t, state))
                        if state != 'G':
                            stdClosed = state
                elif element.hasChild(PERM_STATE):
                    permState = SIGNALSTATE_SUMOSTATE[textNode(element, PERM_STATE)]
                else:
                    raise Exception('Neither switches nor permanent signal state were given. Cannot interpret signal state for sgIndex %s (programID %s)' % (sgID, programID))

                sgIndices = sgIndex[sgID]
                start_index = 0
                assign = ''
                if len(switches) < 1:
                    for sgIdx in sgIndices:
                        for i in range(0, cycleTime):
                            sigProgram[i][sgIdx].append(permState)
                else:
                    for k, (t, state) in enumerate(switches):
                        onset = False

                        if state == 'G':
                            assign = stdClosed
                            onset = True
                        else:
                            assign = 'G'


                        for sgIdx in sgIndices:
                            for i in range(start_index, t):
                                sigProgram[i][sgIdx].append(assign)
                        
                            #if programID == '5' and sgID == 'H10':
                            #    pdb.set_trace()

                            if onset:
                                dur = min(redYellowDur[sgIdx], redYellowDur[sgID])
                                for i in range(t - dur, t):
                                    if i < 0:
                                        i += cycleTime
                                    if i >= cycleTime:
                                        i %= cycleTime
                                    sigProgram[i][sgIdx] = ['u']
                            else:
                                dur = min(yellowDur[sgIdx], yellowDur[sgID])
                                for i in range(t, t + dur):
                                    if i >= cycleTime:
                                        i %= cycleTime
                                    sigProgram[i][sgIdx] = ['y']

                        start_index = t + dur if not onset else t
                        if k >= len(switches)-1:
                            for sgIdx in sgIndices:
                                for i in range(start_index, cycleTime):
                                    sigProgram[i][sgIdx].append(state)
            if not options.noGrouping:
                grouped = [(k, list(g)) for k, g in groupby(sigProgram)]
                timedProgram = [(len(g), k) for k, g in grouped]
            else:
                timedProgram = [(1, g) for g in sigProgram]

            for i, (t, state) in enumerate(timedProgram):
                state = [val if len(val) > 0 else ['o'] for val in state]
                #for k, s in enumerate(state):
                    #state[k] = interpret_complex_state_simple(''.join(s))
                    #print(k, state)
                index2groups = getIndex2Groups(sgIndex)
                majorIndex = options.majorIndex if options.majorIndex is not None else []
                state = normalizeStates(state, None, index2groups, options.minorIndex, majorIndex)
                timedProgram[i] = (t, state)
            programs[programID] = timedProgram
            if options.verbose:
                print('Created program %s' % programID)
                print('\n'.join(str(p) for p in timedProgram))
    return programs


def buildSumoPhasesFromOcit(options):
    # read phase states
    phaseIDs = parsePhaseIDs(options.ocitfile)

    for cycle, majorIndex in buildCycles(list(sorted(phaseIDs)), options.ignorePhases):
        firstMajor = majorIndex[cycle[0]]
        nodeID = ''
        nodeIndex = None
        if firstMajor > 10:
            nodeIndex = firstMajor / 10
            nodeID = str(nodeIndex)
        print("NodeID='%s'" % nodeID, "Cycle=%s" % cycle)
        if nodeIndex in options.ignoreNodes:
            print("Skipping NodeID %s" % nodeIndex)
            continue

        # init signal groups
        sgIndex, yellowDur, redYellowDur, maxIndex, partialNodeSG = parseSignalGroups(options.ocitfile, nodeIndex)
        index2groups = getIndex2Groups(sgIndex)
        if options.verbose:
            print("group -> indices")
            for group in sorted(sgIndex.keys()):
                print(group, sgIndex[group])
            print("\nindex -> groups")
            for index, groups in index2groups.items():
                print(index, groups)

        # read phase states
        phases = parsePhaseStates(options.ocitfile, cycle, sgIndex, maxIndex)
        if options.verbose:
            for phaseID in cycle:
                print(phaseID, [''.join(s) for s in phases[phaseID]])
            print()

        # read phase transitions
        transitions = parseTransitions(options, sgIndex, phases)

        # translate multi-letter states
        for phaseID, state in phases.items():
            #print("before", state)
            phases[phaseID] = ''.join(normalizeStates(state, phaseID, index2groups, options.minorIndex))
            #print("after ",state)
        for (transitionID, fromPhase, toPhase), transition in transitions.items():
            for i, state in enumerate(transition):
                transition[i] = normalizeStates(state, transitionID, index2groups, options.minorIndex)
        #print(ID)
        #print("\n".join(map(str,transition)))

        # check for consistent usage of 'g' and 'G'
        recheckGreenMinor(options, phases, transitions)

        # - add yellow and red-yellow states
        addRedYellow(transitions, phases, yellowDur, redYellowDur, options.verbose)

        # compactify transitions
        timedTransitions = compactifyTransitions(transitions)

        # create sumo logic
        phaseProperties, comments = generateSumoPhases(cycle, majorIndex, options.defaultMinDur,
                sgIndex, phases, timedTransitions)
        yield nodeID, sgIndex, maxIndex, phaseProperties, comments


def main(options):
    with open(options.outfile, 'w') as outf:
        outf.write('<additional>\n')

        if options.usePrograms:

            sgIndex, yellowDur, redYellowDur, maxIndex, partialNodeSG = parseSignalGroups(options.ocitfile, 1)

            signalGroups = [l for sgList in partialNodeSG.values() for l in sgList]

            #for nodeIndex, signalGroups in partialNodeSG.items():

            sgIndex2 = dict([(sg, indices) for (sg, indices) in sgIndex.items() if sg in signalGroups])

            programs = parseSignalPrograms(options, sgIndex2, redYellowDur, yellowDur)
            for pID, program in programs.items():
                outf.write('    <tlLogic id="%s" type="static" programID="%s">\n' % (options.tlsID, pID))
                for (dur, phase) in program:
                    outf.write('        <phase duration="%s" state="%s"/>\n' % (dur, ''.join(''.join(p) for p in phase)))
                outf.write('    </tlLogic>\n')

        else:
            for nodeID, sgIndex, maxIndex, phaseProperties, comments in buildSumoPhasesFromOcit(options):
                # write tls
                writeTLS(outf, options.tlsID + nodeID, options.phaseDuration, sgIndex,
                        maxIndex, phaseProperties, comments)
        outf.write('\n')
        outf.write('</additional>\n')


if __name__ == "__main__":
    if not main(get_options()):
        sys.exit(1)
