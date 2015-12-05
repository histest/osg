#include "stdafx.h"
#include "window.h"
#include <iostream>
using std::cout;
using std::endl;
Window* Window::_instance=NULL;

Window* Window::get_instance()
{
	if(!_instance) _instance = new Window();
	return _instance;
}

Window::Window():_sceneviewer(new osgViewer::Viewer)
{

}
void Window::init(int width,int height)
{
	/** 获得窗口系统的API接口*/
	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();

	/** 创建一个GraphicsContext 的特征表示*/
	osg::ref_ptr<osg::GraphicsContext::Traits> _traits = new osg::GraphicsContext::Traits;

	/** 一个显示器设备一些信息*/
	osg::GraphicsContext::ScreenIdentifier is;
	is.readDISPLAY();
	
	/** get display setting and get the scene width and height*/
	unsigned int scenewidth ;
	unsigned int sceneheight;
	
	wsi->getScreenResolution(is,scenewidth,sceneheight);

	/** set window attitute*/
	_traits->x      = (scenewidth - width)/2;
	_traits->y      = (sceneheight - height)/2;
	_traits->width  = width;
	_traits->height = height;
	_traits->doubleBuffer  = true;
    _traits->sharedContext = 0;
	_traits->windowName    = "OSG_LEARNING";
	_traits->windowDecoration =true;

	//_traits->useCursor = false;

	/** create grahicscontext for viewer camara*/
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(_traits.get());

	if(!gc.valid()){
		string error("Error: can't create graphiscontext!.....");
		throw error;
	}
	_gw = dynamic_cast<osgViewer::GraphicsWindow*>(gc.get());
	if(!_gw.valid())
	{
		string error("Error: can't get the graphisWindow for handle input!...");
		throw error;
	}

	//_gw->setCursor(osgViewer::GraphicsWindow::NoCursor);

	/** set context and viewport*/
	_sceneviewer->getCamera()->setGraphicsContext(gc.get());
	_sceneviewer->getCamera()->setViewport(0,0,width,height);

	/** adjust the projectionmatrix*/
	double fovy,aspect,znear,zfar;
	_sceneviewer->getCamera()->getProjectionMatrixAsPerspective(fovy,aspect,znear,zfar);
	double aspectchange = (double)width/(double)height/aspect;
	if(aspectchange !=1.0)
		_sceneviewer->getCamera()->getProjectionMatrix() *=osg::Matrixd::scale(aspectchange,1.0,1.0);

	/** set the double buffer */
	GLenum buffer = _traits->doubleBuffer ? GL_BACK : GL_FRONT;
    _sceneviewer->getCamera()->setDrawBuffer(buffer);
    _sceneviewer->getCamera()->setReadBuffer(buffer);

	/** realize*/
	_sceneviewer->realize();
	
}
Window::~Window(){
	_instance = NULL;
}
void Window::draw()
{
	/** pre frame operator*/
	preFrameOper();

	/** _call frame*/
	_sceneviewer->frame();

	/** post frame operator*/
	postFrameOper();
}
bool Window::isQuit()
{
	return _sceneviewer->done();
}
void Window::preFrameOper()
{
	/** do something here*/
}
void Window::postFrameOper()
{
	/** do something here*/
}