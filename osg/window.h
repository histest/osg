
#ifndef WINDOW_H
#define WINDOW_H

#include <string>
using std::string;

#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>

class Window{
public:

	static Window* get_instance();
    ~Window();

	void init(int width,int height);

	osgViewer::Viewer* getViewer() { return _sceneviewer.get();}
	osgViewer::GraphicsWindow* getGWindow() {return _gw.get();}

    void draw();
	bool isQuit();
private:
	static Window* _instance;
	
	Window();
	void preFrameOper();
	void postFrameOper();
	/** the host viewer */
	osg::ref_ptr<osgViewer::Viewer> _sceneviewer;
	/**¡¡window for handle input*/
	osg::ref_ptr<osgViewer::GraphicsWindow> _gw;
};
#endif
