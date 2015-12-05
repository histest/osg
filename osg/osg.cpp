#include <stdafx.h>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>


#include "ModelShape.h"
#include "PickHandler.h"


//#pragma comment(lib, "osgd.lib")
//#pragma comment(lib, "osgDBd.lib")
//#pragma comment(lib, "osgGAd.lib")
//#pragma comment(lib, "osgViewerd.lib")
//#pragma comment(lib, "osgManipulatord.lib")


int main(void)
{
	osgViewer::Viewer viewer;

	osg::ref_ptr<osg::Group> root = new osg::Group();

	// build the scene with boxes and gliders.
	for (int i = 1; i < 3; ++i)
	{
		for (int j = 1; j < 3; ++j)
		{
			osg::ref_ptr<osg::MatrixTransform> box = new osg::MatrixTransform();
			osg::ref_ptr<osg::MatrixTransform> glider = new osg::MatrixTransform();

			box->setMatrix(osg::Matrix::translate(i * 6.0, j * 6.0, 0.0));
			glider->setMatrix(osg::Matrix::translate(i * 2.5, j * 2.5, 6.0));

			box->addChild(new ModelShape(osgDB::readNodeFile("gupen1.stl")));
			glider->addChild(new ModelShape(osgDB::readNodeFile("glider.osg")));//addChild(new ModelShape(osgDB::readNodeFile("glider.osg")));

		//	root->addChild(box);
			root->addChild(glider);
		}
	}

	viewer.setSceneData(root.get());

	viewer.addEventHandler(new osgViewer::StatsHandler());
	viewer.addEventHandler(new osgViewer::WindowSizeHandler());
	//viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

	// add pick event handler to add dragger on the shape.
	viewer.addEventHandler(new PickHandler());

	return viewer.run();
}