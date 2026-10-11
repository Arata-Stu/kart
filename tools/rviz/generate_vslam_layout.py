#!/usr/bin/env python3
"""Generate RViz's Qt5 dock state, without ROS/OGRE or modifying the target file.

Run with QT_QPA_PLATFORM=offscreen and a development PyQt5 installation.
Print Window Geometry YAML to paste into config/sim/vslam.rviz.
"""
import sys
from PyQt5.QtCore import Qt, QByteArray
from PyQt5.QtWidgets import QApplication, QMainWindow, QDockWidget, QLabel, QToolBar
import yaml


def window():
    main=QMainWindow();main.setObjectName('VisualizationFrame');main.resize(1600,900)
    main.setCentralWidget(QLabel('3D'))
    toolbar=QToolBar('Tools');toolbar.setObjectName('Tools');main.addToolBar(toolbar)
    docks={}
    for name in ('Displays','Left mono8','Right mono8','Left landmark reprojection','Right landmark reprojection'):
        dock=QDockWidget(name);dock.setObjectName(name)
        panel=QLabel(name);panel.setMinimumSize(220,120);dock.setWidget(panel)
        main.addDockWidget(Qt.LeftDockWidgetArea if name=='Displays' else Qt.RightDockWidgetArea,dock)
        docks[name]=dock
    main.splitDockWidget(docks['Left mono8'],docks['Right mono8'],Qt.Vertical)
    main.tabifyDockWidget(docks['Left mono8'],docks['Left landmark reprojection'])
    main.tabifyDockWidget(docks['Right mono8'],docks['Right landmark reprojection'])
    main.show();QApplication.processEvents()
    main.resizeDocks([docks['Displays'],docks['Left mono8']],[300,400],Qt.Horizontal)
    main.resizeDocks([docks['Left mono8'],docks['Right mono8']],[420,420],Qt.Vertical)
    docks['Left mono8'].raise_();docks['Right mono8'].raise_();QApplication.processEvents()
    return main,docks


def main():
    app=QApplication(sys.argv)
    original,docks=window()
    state=original.saveState()
    geometry={'Width':1600,'Height':900,'X':0,'Y':0,'Hide Left Dock':False,'Hide Right Dock':False,
              'QMainWindow State':bytes(state.toHex()).decode()}
    geometry.update({name:{'collapsed':False} for name in docks})
    # Round-trip with a fresh window: check the encoded names and docking areas.
    restored,panels=window()
    assert restored.restoreState(QByteArray.fromHex(geometry['QMainWindow State'].encode()))
    QApplication.processEvents()
    assert restored.centralWidget().width()>restored.width()/2
    assert panels['Displays'].width()<restored.centralWidget().width()
    assert all(not dock.isHidden() for dock in panels.values())
    assert min(panels['Left mono8'].height(),panels['Right mono8'].height())>200
    assert restored.dockWidgetArea(panels['Displays'])==Qt.LeftDockWidgetArea
    for name in panels:
        if name!='Displays':assert restored.dockWidgetArea(panels[name])==Qt.RightDockWidgetArea
    assert panels['Left landmark reprojection'] in restored.tabifiedDockWidgets(panels['Left mono8'])
    assert panels['Right landmark reprojection'] in restored.tabifiedDockWidgets(panels['Right mono8'])
    print(yaml.safe_dump({'Window Geometry':geometry},sort_keys=False),end='')
    original.close();restored.close()


if __name__=='__main__':main()
