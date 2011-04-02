#!/usr/bin/env python
# -*- Python -*-
# -*- coding: utf-8 -*-

'''BlocksWorld simulator

Copyright (C) 2011
    Yosuke Matsusaka
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''
#  special thanks to Fumio Kanehiro

import os
import sys
if not hasattr(sys, "frozen"):
    _openhrp_idl_path = os.path.join(os.path.dirname(__file__), 'idl')
    if _openhrp_idl_path not in sys.path:
        sys.path.append(_openhrp_idl_path)
import time
import traceback
import math
import threading
from omniORB import CORBA
import CosNaming
import OpenHRP
import OpenRTM_aist
import RTC
from pprint import pprint

class BlocksWorld(threading.Thread):
    def __init__(self, orb):
        threading.Thread.__init__(self)
        if hasattr(sys, "frozen"):
            self._basedir = os.path.dirname(unicode(sys.executable, sys.getfilesystemencoding()))
        else:
            self._basedir = os.path.dirname(__file__)
        self._basedir = os.path.abspath(self._basedir)
        self._basedir = self._basedir.replace('\\', '/')

        # define constants
        self.ROBOT_URL = "file://"+self._basedir+"/PA10/pa10.main.wrl"
        self.FLOOR_URL = "file://"+self._basedir+"/floor.wrl"
        self.BOX1_URL = "file://"+self._basedir+"/box.wrl"
        self.BOX2_URL = "file://"+self._basedir+"/box2.wrl"
        self.BOX3_URL = "file://"+self._basedir+"/box3.wrl"

        self.time = 0.0
        self.dt = 0.01
        self.prevjoint = None
        
        self.commands = []

        # find CORBA name server
        self.ns = OpenRTM_aist.CorbaNaming(orb, 'localhost:2809')
        
        # find OpenHRP components from the name server
        simfactoryobj = self.ns.resolve([CosNaming.NameComponent("DynamicsSimulatorFactory","")])
        self.simfactory = simfactoryobj._narrow(OpenHRP.DynamicsSimulatorFactory)
        modelloaderobj = self.ns.resolve([CosNaming.NameComponent("ModelLoader","")])
        self.modelloader = modelloaderobj._narrow(OpenHRP.ModelLoader)
        viewerobj = self.ns.resolve([CosNaming.NameComponent("OnlineViewer","")])
        self.viewer = viewerobj._narrow(OpenHRP.OnlineViewer)
        
        # load models from the file
        self.robot = self.modelloader.loadBodyInfo(self.ROBOT_URL)
        self.floor = self.modelloader.loadBodyInfo(self.FLOOR_URL)
        self.box1 = self.modelloader.loadBodyInfo(self.BOX1_URL)
        self.box2 = self.modelloader.loadBodyInfo(self.BOX2_URL)
        self.box3 = self.modelloader.loadBodyInfo(self.BOX3_URL)

        # initialize real world dynamics simulator
        self.sim = self.simfactory.create()
        self.sim.registerCharacter("robot", self.robot)
        self.sim.registerCharacter("floor", self.floor)
        self.sim.registerCharacter("box1", self.box1)
        self.sim.registerCharacter("box2", self.box2)
        self.sim.registerCharacter("box3", self.box3)

        # initialize virtual world dynamics simulator
        self.planner = self.simfactory.create()
        self.planner.registerCharacter("robot", self.robot)
        
        # set simulation algorithms
        self.sim.init(self.dt,
                      OpenHRP.DynamicsSimulator.RUNGE_KUTTA,
                      OpenHRP.DynamicsSimulator.DISABLE_SENSOR)
        self.planner.init(self.dt,
                          OpenHRP.DynamicsSimulator.RUNGE_KUTTA,
                          OpenHRP.DynamicsSimulator.DISABLE_SENSOR)
        
        # set gravity
        self.sim.setGVector([0, 0, 9.8])
        self.planner.setGVector([0, 0, 9.8])

        # set initial joint angles
        self.sim.setCharacterAllJointModes("robot", OpenHRP.DynamicsSimulator.TORQUE_MODE)
        self.planner.setCharacterAllJointModes("robot", OpenHRP.DynamicsSimulator.HIGH_GAIN_MODE)

        for s in [self.sim, self.planner]:
            s.setCharacterLinkData("robot", "J2", OpenHRP.DynamicsSimulator.JOINT_VALUE, [math.pi/3])
            s.setCharacterLinkData("robot", "J4", OpenHRP.DynamicsSimulator.JOINT_VALUE, [math.pi/3])
            s.setCharacterLinkData("robot", "J6", OpenHRP.DynamicsSimulator.JOINT_VALUE, [math.pi/3])
            s.setCharacterLinkData("robot", "HAND_L", OpenHRP.DynamicsSimulator.JOINT_VALUE, [-0.04])
            s.setCharacterLinkData("robot", "HAND_R", OpenHRP.DynamicsSimulator.JOINT_VALUE, [0.04])
            s.setCharacterLinkData("box1", "WAIST", OpenHRP.DynamicsSimulator.ABS_TRANSFORM, [0.82, 0.0, 0.135, 1,0,0,0,1,0,0,0,1])
            s.setCharacterLinkData("box2", "WAIST", OpenHRP.DynamicsSimulator.ABS_TRANSFORM, [0.62, 0.1, 0.135, 1,0,0,0,1,0,0,0,1])
            s.setCharacterLinkData("box3", "WAIST", OpenHRP.DynamicsSimulator.ABS_TRANSFORM, [0.82, 0.2, 0.135, 1,0,0,0,1,0,0,0,1])
            s.calcWorldForwardKinematics()

        for o in ["box1", "box2", "box3"]:
            self.sim.registerCollisionCheckPair("floor", "", o, "", 0.5, 0.5, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], 0.01)
            for p in ["HAND_L", "HAND_R"]:
                self.sim.registerCollisionCheckPair("robot", p, o, "", 0.5, 0.5, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], 0.01)
        for o in ["HAND_L", "HAND_R", "J7"]:
            self.sim.registerCollisionCheckPair("floor", "", "robot", o, 0.5, 0.5, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], 0.01)
        self.sim.registerCollisionCheckPair("robot", "HAND_L", "robot", "HAND_R", 0.5, 0.5, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], 0.01)

        self.sim.initSimulation()
        self.planner.initSimulation()

        # init online viewer
        self.viewer.load("robot", self.ROBOT_URL)
        self.viewer.load("floor", self.FLOOR_URL)
        self.viewer.load("box1", self.BOX1_URL)
        self.viewer.load("box2", self.BOX2_URL)
        self.viewer.load("box3", self.BOX3_URL)
        self.viewer.clearLog()

        self.prevtime = time.clock()

    def run(self):
        # demonstrative movement
        self.moveHandZ(0.2, 2)
        self.moveJoint("J1", math.pi/2, 2.0)
        
        self.moveHandToBox("box1", 4.0)
        
        self.moveHandZ(-0.18, 2)    
        self.grasp()
        self.moveHandZ(0.18, 2)
        
        self.moveHandXY(-0.3, 0.0, 4.0)
        
        self.moveHandZ(-0.18, 2)
        self.release()
        self.moveHandZ(0.18, 2)
        
        while True:
            if len(self.commands) > 0:
                cmdline = self.commands[0]
                self.commands = self.commands[1:]
                cmd = cmdline.split(" ")
                if cmd[0] == "grasp":
                    self.moveHandZ(-0.18, 2)    
                    self.grasp()
                    self.moveHandZ(0.18, 2)
                elif cmd[0] == "moveHandToBox":
                    if cmd[1] in ("box1", "box2", "box3"):
                        self.moveHandToBox(cmd[1], 4.0)
                elif cmd[0] == "moveHandXY":
                    self.moveHandXY(float(cmd[1]), float(cmd[2]), 4.0)
                elif cmd[0] == "release":
                    self.moveHandZ(-0.18, 2)
                    self.release()
                    self.moveHandZ(0.18, 2)
                else:
                    print "unknown command :%s" % (cmd[0],)
            else:
                self.updateView()

    def getLinkPosition(self, char, link):
        lp = OpenHRP.LinkPosition(None, None)
        ldata = self.sim.getCharacterLinkData(char, link, OpenHRP.DynamicsSimulator.ABS_TRANSFORM)
        lp.p = ldata[0:3]
        lp.R = ldata[3:12]
        return lp
    
    def getHandPosition(self):
        return self.getLinkPosition("robot", "J7")

    def getJointAngle(self, joint):
        ldata = self.sim.getCharacterLinkData("robot", joint, OpenHRP.DynamicsSimulator.JOINT_VALUE)
        return ldata[0]

    def setJointAngle(self, joint, angle):
        self.planner.setCharacterLinkData("robot", joint, OpenHRP.DynamicsSimulator.JOINT_VALUE, [angle])
        return

    def limiter(self, v, limit):
        if v > limit:
            v = limit
        if v < -limit:
            v = -limit
        return v
    
    def updateView(self):
        pgain = [8000] * 7 + [1250] * 2
        dgain = [400] * 7 + [20] * 2
        limit = 1500
        
        self.planner.calcWorldForwardKinematics()
        now = self.sim.getCharacterAllLinkData("robot", OpenHRP.DynamicsSimulator.JOINT_VALUE)
        tgt = self.planner.getCharacterAllLinkData("robot", OpenHRP.DynamicsSimulator.JOINT_VALUE)
        torque = [0.0] * len(now)
        if self.prevjoint is None:
            self.prevjoint = [0.0] * len(now)
            for i in range(len(now)):
                self.prevjoint[i] = now[i]
        for i in range(len(now)):
            torque[i] = - (now[i] - tgt[i]) * pgain[i] - (now[i] - self.prevjoint[i]) / self.dt * dgain[i]
            torque[i] = self.limiter(torque[i], limit)
            self.prevjoint[i] = now[i]
        self.sim.setCharacterAllLinkData("robot", OpenHRP.DynamicsSimulator.JOINT_TORQUE, torque)
        self.sim.calcWorldForwardKinematics()
        self.sim.stepSimulation()
        state = self.sim.getWorldState()
        self.viewer.update(state)
        self.time = state.time
        now = time.clock()
        rtdiff = self.dt - (now - self.prevtime)
        if rtdiff > 0:
            time.sleep(rtdiff)
        self.prevtime = now

    def moveHandZ(self, dz, tm):
        nframe = int(tm/self.dt)
        delta = dz/nframe
        lp = self.getHandPosition()
        for i in range(nframe):
            lp.p[2] += delta
            if self.planner.calcCharacterInverseKinematics("robot", "BASE", "J7", lp) == 0:
                print "calcCharacterInverseKinematics() failed"
            self.updateView()
        return

    def moveHandXY(self, dx, dy, tm):
        nframe = int(tm/self.dt)
        deltax = dx/nframe
        deltay = dy/nframe
        lp = self.getHandPosition()
        for i in range(nframe):
            lp.p[0] += deltax
            lp.p[1] += deltay
            if self.planner.calcCharacterInverseKinematics("robot", "BASE", "J7", lp) == 0:
                print "calcCharacterInverseKinematics() failed"
            self.updateView()
        return

    def moveHandToBox(self, name, tm):
        nframe = int(tm/self.dt)
        lp = self.getHandPosition()
        boxlp = self.getLinkPosition(name, "WAIST")
        dx = boxlp.p[0] - lp.p[0]
        dy = boxlp.p[1] - lp.p[1]
        self.moveHandXY(dx, dy, tm)
        return
    
    def moveJoint(self, joint, angle, tm):
        current = self.getJointAngle(joint)
        nframe = int(tm/self.dt)
        dth = (angle - current)/nframe
        for i in range(nframe):
            self.setJointAngle(joint, current + i*dth)
            self.updateView()
        return 

    def moveGripper(self, angle, tm):
        current = self.getJointAngle("HAND_R")
        nframe = int(tm/self.dt)
        dth = (angle - current)/nframe
        for i in range(nframe):
            self.setJointAngle("HAND_L", -(current + i*dth))
            self.setJointAngle("HAND_R", current + i*dth)
            self.updateView()
    
    def grasp(self):
        self.moveGripper(0.00, 0.5)
        return

    def release(self):
        self.moveGripper(0.04, 0.5)
        return

    def textCommand(self, cmdline):
        self.commands.append(cmdline)

class DataListener(OpenRTM_aist.ConnectorDataListenerT):
    def __init__(self, name, obj):
        self._name = name
        self._obj = obj
    
    def __call__(self, info, cdrdata):
        data = OpenRTM_aist.ConnectorDataListenerT.__call__(self, info, cdrdata, RTC.TimedString(RTC.Time(0,0),""))
        self._obj.onData(self._name, data)

BlocksWorld_spec = ["implementation_id", "BlocksWorld",
                    "type_name",         "BlocksWorld",
                    "description",       "OpenHRP blocks world",
                    "version",           "1.0.0",
                    "vendor",            "AIST",
                    "category",          "simulation",
                    "activity_type",     "DataFlowComponent",
                    "max_instance",      "1",
                    "language",          "Python",
                    "lang_type",         "script",
                    ""]

class BlocksWorldRTC(OpenRTM_aist.DataFlowComponentBase):
    def __init__(self, manager):
        try:
            OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        except:
            print traceback.format_exc()

    def onInitialize(self):
        try:
            OpenRTM_aist.DataFlowComponentBase.onInitialize(self)
            self._j = BlocksWorld(OpenRTM_aist.Manager.instance().getORB())
            self._j.start()
            # create inport
            self._indata = RTC.TimedString(RTC.Time(0,0), u"")
            self._inport = OpenRTM_aist.InPort("command", self._indata)
            self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE,
                                                  DataListener("ON_BUFFER_WRITE", self))
            self.addInPort(self._inport._name, self._inport)
        except:
            print traceback.format_exc()
        return RTC.RTC_OK

    def onData(self, name, data):
        try:
            self._j.textCommand(data.data)
        except:
            print traceback.format_exc()

class BlocksWorldRTCManager:
    def __init__(self):
        self._comp = None
        self._manager = OpenRTM_aist.Manager.init(sys.argv)
        self._manager.setModuleInitProc(self.moduleInit)
        self._manager.activateManager()

    def start(self):
        self._manager.runManager(False)

    def moduleInit(self, manager):
        profile=OpenRTM_aist.Properties(defaults_str=BlocksWorld_spec)
        manager.registerFactory(profile, BlocksWorldRTC, OpenRTM_aist.Delete)
        self._comp = manager.createComponent("BlocksWorld")

def main():
    manager = BlocksWorldRTCManager()
    manager.start()

if __name__=='__main__':
    main()

