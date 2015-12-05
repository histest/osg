#include "stdafx.h"
#include <iostream>
using namespace std;
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/LineWidth>
#include <osg/Texture>
#include <osgDB/ReadFile>
#include <osg/Group>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/api/win32/GraphicsWindowWin32>
#include <osgDB/ReadFile>
#include <osgGA/OrbitManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/CameraManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/GUIEventHandler>
#include <osgUtil/Optimizer>
#include <iostream>
#include <sstream>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/Geode>
#include <osg/Depth>
#include <osg/CameraNode>
#include <osgText/Text>
#include <osgGA/TrackballManipulator>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgGA/CameraManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/WriteFile>
#include <winsock2.h>
#include <osg/ShapeDrawable>
#include <osg/Shape>
#include "window.h"
osg::Node* createTrans()
{
	/** declare a root node*/
	osg::Group* root = new osg::Group;
	/** declare a Position Node*/
	osg::PositionAttitudeTransform* posCow = new osg::PositionAttitudeTransform;
	root->addChild(posCow);
	/** declare a Matrix Node*/
	osg::MatrixTransform* matrixCow  = new osg::MatrixTransform;
	root->addChild(matrixCow);

	osg::MatrixTransform* initCow  = new osg::MatrixTransform;
	root->addChild(initCow);

	osg::Node* cow = osgDB::readNodeFile("cow.osg");

	/** 
		When use Position Node and the ReferenceFrame is RELATIVE_RF
		the matrix is Compute  Trans(-pivot) * scale * Rotate * Trans(Pos)
		here the pivot and scale is default,so it means that make rotate firstly.
	*/
	posCow->addChild(cow);
	osg::Quat quat,quat1;
	quat.makeRotate(osg::PI_2,osg::Vec3(0.0,0.0,1.0));
	posCow->setAttitude(quat);
	posCow->setPosition(osg::Vec3(-10,0.0,0.0));

	/**
		when use Matrix Node  you can set the matrix what you want.
		here , it  make trans firstly and then make rotate.
	*/
	matrixCow->addChild(cow);
	quat.makeRotate(osg::DegreesToRadians(60.0),osg::Vec3(0.0,0.0,1.0));
	quat.makeRotate(osg::PI_2,osg::Vec3(0.0,0.0,1.0));
	quat1.makeRotate(osg::PI,osg::Vec3(0.0,0.0,1.0));
	matrixCow->setMatrix(osg::Matrixd::rotate(quat)*osg::Matrixd::translate(osg::Vec3(-10.0,0.0,0.0))*osg::Matrixd::rotate(quat1));

	initCow->addChild(cow);
	return root;
}

int emain()
{
	Window* win = Window::get_instance();

	try{
		win->init(800,600);
	}
	catch(string& error){
		cout<<error<<endl;
	}
	
	/** 
		set the default Camera Manapulator
		it seems that if you use viewer.run(),when the camera Manipulator is NULL, 
		it will be setted  by viewer.run() function.
	*/
	win->getViewer()->setSceneData(createTrans());
	win->getViewer()->setCameraManipulator(new osgGA::TrackballManipulator);

	while(!win->isQuit())
	{
		win->draw();
	}
	delete win;
	return 0;
}