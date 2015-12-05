// osgPlugin.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
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
#include <osg/ShapeDrawable>
#include <osg/Shape>
#include <osg/Camera>
#include <conio.h>
#include "UserMechanics.h"
#include <osg/PositionAttitudeTransform>
#include "window.h"
osg::ref_ptr<osgViewer::Viewer> viewer; 
osg::ref_ptr<osg::Group> root;
osg::ref_ptr< osg::Node> model;
osg::ref_ptr< osg::Node> model2;
static double Rpt0[3][3]={0.0};//骨头上点在tracker系姿态
static double Rsp_right[3][3]={0.0};//模型上点在模型坐标系下姿态(固定值)
static double Rot0[3][3]={0.0};//
static double Rbo0[3][3]={0.0};//
static double Rpo0[3][3]={0.0};//模型上三点在osg下姿态

static double Rpt[3][3]={0.0};//骨头上点在tracker系姿态
static double Rsp_left[3][3]={0.0};//模型上点在模型坐标系下姿态(固定值)
static double Rot[3][3]={0.0};//
static double Rbo[3][3]={0.0};//
static double Rpo[3][3]={0.0};//模型上三点在osg下姿态
static double deltaPosition[3]={0.0};//	
using namespace std;

 osg::ref_ptr<osg::PositionAttitudeTransform> rightpat; 
 osg::ref_ptr<osg::PositionAttitudeTransform> leftpat;
 double Ltb_tracker_right[3]={0.0};//tracker原点与骨头质心在tracker下
double Ltb_tracker_left[3]={0.0};//tracker原点与骨头质心在tracker下
//#define direct  5
 void RealTimeTracking(int Scale,const double Attitude[3],const double Position[3],const double Attitude2[3],const double Position2[3])
{
	double PositionInWorld[3];
	double AttitudeInWorld[3];
	double Position2InWorld[3];
	double Attitude2InWorld[3];
	double Rtc[3][3],Rbc[3][3];
	double Rct[3][3];
	struct SEularAngle  Angle;
	double Rtc0[3][3];

	double Q0[4]={0.0};
	double Rco0[3][3]={ { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } };
	double Rto0[3][3]={0.0};//tracker在osg坐标系下的姿态
	double Rpo0[3][3]={0.0};//骨头上标定点在osg坐标系姿态
	double Rbo0[3][3]={0.0};//骨头在osg坐标系姿态
	double relative_position_right[3];
	double relative_position_left[3];


	for (int i =0;i<4;i++)
		Q0[i] = Attitude[i];
	QuaternionToMatrix(Q0,Rtc0);//tracker在相机坐标系下的姿态
	
	MatrixProduct(3,3,3,&Rtc0[0][0],&Rco0[0][0],&Rto0[0][0]);
	MatrixProduct(3,3,3,&Rpt0[0][0],&Rto0[0][0],&Rpo0[0][0]);
	MatrixProduct(3,3,3,&Rsp_right[0][0],&Rpo0[0][0],&Rbo0[0][0]);
	QuaternionExtract(Rbo0,Q0);
	Eular313Extract(Rbo0,&Angle);

	osg::Quat quat0;

	quat0._v[3]=Q0[0];
	quat0._v[0]=Q0[1];
	quat0._v[1]=Q0[2];
	quat0._v[2]=Q0[3];
	rightpat->setAttitude(quat0);
#if direct//直接用三轴转角
	osg::Quat quat0(Angle.Pitch,osg::Vec3d(0.0, 1.0, 0.0),Angle.Roll,osg::Vec3d(1.0, 0.0, 0.0),Angle.Yaw,osg::Vec3d(0.0, 0.0, 1.0));
	rightpat->setAttitude(quat0);
#endif	

	//加上相对位置
	MatrixTranspose(3,3,&Rtc0[0][0],&Rct[0][0]);
	MatrixProduct(3,3,1,&Rct[0][0],Ltb_tracker_right,relative_position_right);
	for (int i=0;i<3;i++)
	{
		relative_position_right[i]=0.0;
	}
	rightpat->setPosition(osg::Vec3d( Position[0]+relative_position_right[0],Position[1]+relative_position_right[1], Position[2]+relative_position_right[2]));


	double Q[4]={0.0};
	double Rco[3][3]={ { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } };
	double Rto[3][3]={0.0};//tracker在osg坐标系下的姿态
	double Rpo[3][3]={0.0};//骨头上标定点在osg坐标系姿态
	double Rbo[3][3]={0.0};//骨头在osg坐标系姿态
	//double Rsp[3][3]={0.0};

	for (int i =0;i<4;i++)
		Q[i] = Attitude2[i];
	QuaternionToMatrix(Q,Rtc);//tracker在相机坐标系下的姿态
	//QuaternionExtract(Rtc,Q);
	MatrixProduct(3,3,3,&Rtc[0][0],&Rco[0][0],&Rto[0][0]);
	MatrixProduct(3,3,3,&Rpt[0][0],&Rto[0][0],&Rpo[0][0]);
	MatrixProduct(3,3,3,&Rsp_left[0][0],&Rpo[0][0],&Rbo[0][0]);

	QuaternionExtract(Rbo,Q);
	Eular313Extract(Rbo,&Angle);

	osg::Quat quat;
	quat._v[3]=Q[0];
	quat._v[0]=Q[1];
	quat._v[1]=Q[2];
	quat._v[2]=Q[3];
	double angle =90;
	osg::Quat quat_a =quat.inverse()*quat0;//= osg::Quat(0, osg::Y_AXIS,osg::DegreesToRadians(angle), osg::X_AXIS,0, osg::Z_AXIS);
	double length = quat_a.length();
	osg::Quat quat_b = quat*quat_a;
	leftpat->setAttitude(quat);
#if direct//直接用三轴转角
	osg::Quat quat1(Angle.Pitch,osg::Vec3d(0.0, 1.0, 0.0),Angle.Roll,osg::Vec3d(1.0, 0.0, 0.0),Angle.Yaw,osg::Vec3d(0.0, 0.0, 1.0));
	leftpat->setAttitude(quat1);
#endif

	//加上相对位置
	MatrixTranspose(3,3,&Rtc[0][0],&Rct[0][0]);
	MatrixProduct(3,3,1,&Rct[0][0],Ltb_tracker_left,relative_position_left);
	for (int i=0;i<3;i++)
	{
		relative_position_left[i]=0.0;
	}
	leftpat->setPosition(osg::Vec3d( Position2[0]+relative_position_left[0],Position2[1]+relative_position_left[1], Position2[2]+relative_position_left[2]));
 }
 void  GetTransformMatrix(const int id, const double P1[3],const double P2[3],const double Attitude[3],const double R1[3],const double R2[3],const double V1[3],const double V2[3],double M1[3][3],double M2[3][3])
 {
	 //P1为骨头实物上点在tracker下坐标系，P2为骨头模型上点在OSG坐标系下坐标，O为Tracker坐标系原点，O'为OSG坐标系原点
	 //OP1为骨头实物上点到原点的向量（tracker坐标系），到投影到OSG坐标系下
	 //O'P2为骨头模型上点OSG原点的向量

	 //标定的时候能否得到Tracker姿态

	 //实物上某一点与tracker坐标系原点的位置矢量ox，得到ox在相机系下的投影ox'，对应的该点在osg下的位置o'x'，骨头质心在osg下的位置o'b
	 //ob= ox'+x'b  tracker与骨头质心距离在osg/相机下的投影，将它投影到tracker坐标系下，为常值。

	 //以后得到tracker姿态，得到相对位置在相机系投影，加上tracker的位置，认为是骨头质心位置。


	 double Rbi[3][3] = {0.0},Rei[3][3] = {0.0},Rop[3][3] = {0.0};

	 struct SEularAngle  Angle;
	 double Rtc[3][3];
	 QuaternionToMatrix(Attitude,Rtc);//tracker在相机坐标系下的姿态
	 if(id==0)
	 {
		 GetRdi(R1,R2,Rpt0);
		 GetRdi(V1,V2,Rpo0);
		 //模型在OSG下姿态
		 double Q[4]={0.0};
		 osg::Quat quat = rightpat->getAttitude();
		 Q[0]=quat._v[3];//w();
		 Q[1]=quat._v[0];//x();
		 Q[2]=quat._v[1];//y();
		 Q[3]=quat._v[2];//z();
		 QuaternionToMatrix(Q,Rbo0);
		 MatrixTranspose(3,3,&Rpo0[0][0],&Rop[0][0]);
		 MatrixProduct(3,3,3,&Rbo0[0][0],&Rop[0][0],&Rsp_right[0][0]);


		 //位置的处理 P1为实物选点，P2为osg对应的点
		 double Rct[3][3]={0.0};
		 double Ltp_camera[3]={0.0};//tracker原点与p1位置在相机系下
		 double bone_osg[3]={0.0};//骨头质心在osg下的位置
		 double Lpb_osg[3]={0.0};//点与骨头质心在osg下的相对位置
		 double Ltb_osg[3]={0.0};//tracker原点与骨头质心在osg下


		 MatrixTranspose(3,3,&Rtc[0][0],&Rct[0][0]);
		 MatrixProduct(3,3,1,&Rct[0][0],P1,Ltp_camera);
		 osg::Vec3 position_osg = rightpat->getPosition();
		 bone_osg[0]=position_osg._v[0];
		 bone_osg[1]=position_osg._v[1];
		 bone_osg[2]=position_osg._v[2];
		 for (int i=0;i<3;i++)
		 {
			 Lpb_osg[i]= bone_osg[i]-P2[i];
		 }
		 for (int i=0;i<3;i++)
		 {
			 Ltb_osg[i]= Ltp_camera[i]+Lpb_osg[i];
		 }
		 MatrixProduct(3,3,1,&Rtc[0][0],Ltb_osg,Ltb_tracker_right);
		std::cout << "Node is rotate around the axis(" << Ltb_tracker_right[0] <<Ltb_tracker_right[1] <<Ltb_tracker_right[2] <<")" << std::endl;
	 }
	 if(id==1)
	 {
		 GetRdi(R1,R2,Rpt);
		 GetRdi(V1,V2,Rpo);
		 //模型在OSG下姿态
		 double Q[4]={0.0};
		 osg::Quat quat = leftpat->getAttitude();
		 Q[0]=quat._v[3];//w();
		 Q[1]=quat._v[0];//x();
		 Q[2]=quat._v[1];//y();
		 Q[3]=quat._v[2];//z();
		 QuaternionToMatrix(Q,Rbo);

		 MatrixTranspose(3,3,&Rpo[0][0],&Rop[0][0]);
		 MatrixProduct(3,3,3,&Rbo[0][0],&Rop[0][0],&Rsp_left[0][0]);



		 //位置的处理 P1为实物选点，P2为osg对应的点
		 double Rct[3][3]={0.0};
		 double Ltp_camera[3]={0.0};//tracker原点与p1位置在相机系下
		 double bone_osg[3]={0.0};//骨头质心在osg下的位置
		 double Lpb_osg[3]={0.0};//点与骨头质心在osg下的相对位置
		 double Ltb_osg[3]={0.0};//tracker原点与骨头质心在osg下


		 MatrixTranspose(3,3,&Rtc[0][0],&Rct[0][0]);
		 MatrixProduct(3,3,1,&Rct[0][0],P1,Ltp_camera);
		 osg::Vec3 position_osg = leftpat->getPosition();
		 bone_osg[0]=position_osg._v[0];
		 bone_osg[1]=position_osg._v[1];
		 bone_osg[2]=position_osg._v[2];
		 for (int i=0;i<3;i++)
		 {
			 Lpb_osg[i]= bone_osg[i]-P2[i];
		 }
		 for (int i=0;i<3;i++)
		 {
			 Ltb_osg[i]= Ltp_camera[i]+Lpb_osg[i];
		 }
		 MatrixProduct(3,3,1,&Rtc[0][0],Ltb_osg,Ltb_tracker_left);
	 }
	 // MatrixProduct(3,3,3,&Rop[0][0],&Rpt[0][0],&Rot[0][0]);

	 for (int i=0;i<3;i++)
	 {
		 for (int j=0;j<3;j++)
		 {
			 M1[1][j]=Ltb_tracker_right[j];
		 }
	 }

	 for (int i=0;i<3;i++)
	 {
		 for (int j=0;j<3;j++)
		 {
			 M2[1][j]=Ltb_tracker_left[j];
		 }
	 }
 }
class RightRotateCallBack: public osg::NodeCallback
{
public:
	RightRotateCallBack():_rotateZ(0.0) {}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::PositionAttitudeTransform* pat = 
			dynamic_cast<osg::PositionAttitudeTransform*>(node);
		if(pat)
		{
			osg::Vec3 vec(0, 0, 1);
			double position[3];
		//	pat->setPosition(osg::Vec3d(_rotateZ*sin(_rotateZ/1000),0,0));
			
			osg::Matrix mat;
			mat = osg::computeLocalToWorld(nv->getNodePath());	
			osg::Quat quat0 = mat.getRotate();
			std::cout << "Node's attitude is:(" << mat.getRotate().w()<< mat.getRotate().x() <<mat.getRotate().y() <<mat.getRotate().z() <<")" << std::endl;

			double yangle = 60;
			osg::Quat quat = osg::Quat(osg::DegreesToRadians(yangle), osg::Y_AXIS);
			osg::Quat quat2 = osg::Quat(osg::DegreesToRadians(_rotateZ), osg::X_AXIS);
			/*osg::Quat q(yangle*ToRad,osg::Vec3d(0.0, 1.0, 0.0),yangle*ToRad,osg::Vec3d(1.0, 0.0, 0.0),_rotateZ*ToRad,osg::Vec3d(0.0, 0.0, 1.0));*/
		//	pat->setAttitude(q);
			_rotateZ += 0.50;
		}

		traverse(node, nv);
	}

private:
	double _rotateZ;
	double _rotatex;
};
double rightangle;
class LeftRotateCallBack: public osg::NodeCallback
{
public:
	LeftRotateCallBack():_rotateZ(0.0) {}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::PositionAttitudeTransform* pat = 
			dynamic_cast<osg::PositionAttitudeTransform*>(node);
		if(pat)
		{
			osg::Vec3 vec(0, 0, 1);

			/*	osg::Quat quat = osg::Quat(osg::DegreesToRadians(_rotateZ), osg::Y_AXIS);
			pat->setAttitude(quat);
			_rotatex=34;
			osg::Quat quat2 = osg::Quat(osg::DegreesToRadians(_rotatex), osg::X_AXIS);	
			pat->setAttitude(quat2*quat);
			double position[3];
			pat->setPosition(osg::Vec3d(_rotateZ*sin(_rotateZ/1000),0,0));
			_rotateZ = 90;*/
		}

		traverse(node, nv);
	}

private:
	double _rotateZ;
	double _rotatex;
};
class InfoCallBack: public osg::NodeCallback
{
public:
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::PositionAttitudeTransform* pat =
			dynamic_cast<osg::PositionAttitudeTransform*>(node);

		if(pat)
		{
			double angle = 0.0;
			osg::Vec3 axis;
			pat->getAttitude().getRotate(angle, axis);
			
			//std::cout << "Node is rotate around the axis(" << axis << "), "
			//	<<osg::RadiansToDegrees(angle) << "degrees" << std::endl;
		}

		traverse(node, nv);
	}
};
void action()
{
	while (1)
	{		
		Sleep(10);
		//double yangle = 60;
		//osg::Quat quat = osg::Quat(osg::DegreesToRadians(yangle), osg::Y_AXIS);
		//osg::Quat quat2 = osg::Quat(osg::DegreesToRadians(rightangle), osg::X_AXIS);
		//osg::Quat q(yangle,osg::Vec3d(0.0, 1.0, 0.0),rightangle,osg::Vec3d(1.0, 0.0, 0.0),30,osg::Vec3d(0.0, 0.0, 1.0));
		//rightpat->setAttitude(q);
	//	 double Attitude[4] = { 1.0,0.0,0.0,0.0};
		 double Position [3]= { 0.0};
	//	 double Attitude2[4] = { 1.0,0.0,0.0,0.0};
		 double Position2[3] = { 0.0};
		 double angle =90;
		osg::Quat quat = osg::Quat(osg::DegreesToRadians(rightangle), osg::X_AXIS,osg::DegreesToRadians(angle), osg::Y_AXIS,0, osg::Z_AXIS);
		//Attitude[0]=quat.w();
		//Attitude[1]=quat.x();
		//Attitude[2]=quat.y();
		//Attitude[3]=quat.z();

		double Attitude[4] = { 0.592,-0.716,-0.099,-0.357};
		double Attitude2[4] = { 0.623,0.659,-0.035,0.420};

		RealTimeTracking(1,Attitude, Position,Attitude2,Position2);
		double Q[4]={0.0};
		quat = rightpat->getAttitude();
		osg::Vec3 posi = rightpat->getPosition();
		rightangle+=0.1;
		if (rightangle>20000)
		{
			rightangle=0;
		}
	}

}
int main(void )
{
	root= new osg::Group; 
	rightpat = new osg::PositionAttitudeTransform();
	rightpat->setReferenceFrame( osg::Transform::RELATIVE_RF );
	leftpat = new osg::PositionAttitudeTransform();
	leftpat->setReferenceFrame( osg::Transform::RELATIVE_RF );

    osg::Node* rightmodel =  osgDB::readNodeFile("test_you2_001.stl") ;//("cow.osg");//

    rightpat->addChild(rightmodel);
    rightpat->setUpdateCallback(new RightRotateCallBack() );

	osg::Node* leftmodel =  osgDB::readNodeFile("test_zuo1_001.stl") ;//("cow.osg");//

	//rightpat->addChild(leftmodel);
	leftpat->addChild(leftmodel);
	leftpat->setUpdateCallback(new LeftRotateCallBack() );


	root->addChild(rightpat.get());
	root->addChild(leftpat.get());


		double Point1[3] = { 0.0, 0.0, 0.0 };
		double Point2[3]  = { 0.0, 0.0, 0.0 };
		double Point3 [3] = { 0.0, 0.0, 0.0 };
	
		double SelectPoint1[3] = { 0.0, 0.0, 0.0 };
		double SelectPoint2[3]  = { 0.0, 0.0, 0.0 };
		double SelectPoint3[3]  = { 0.0, 0.0, 0.0 };
		double R1[3]  = { 0.0, 0.0, 0.0 };
	    double R2[3]  = { 0.0, 0.0, 0.0 };
	    double V1[3]  = { 0.0, 0.0, 0.0 };
	    double V2[3]  = { 0.0, 0.0, 0.0 };
	
	
		Point1[0] = 101;
		Point1[1] = 260;
		Point1[2] = -45;


		Point2[0] = 140;
		Point2[1] = 266;
		Point2[2] = -47;

		Point3[0] = 155;
		Point3[1] = 242;
		Point3[2] = -111;


		SelectPoint1[0] = -76;
		SelectPoint1[1] = 6;
		SelectPoint1[2] = 8;
		SelectPoint2[0] = -78;
		SelectPoint2[1] = 0;
		SelectPoint2[2] = 48;
		SelectPoint3[0] = -146;
		SelectPoint3[1] = -3;
		SelectPoint3[2] = 57;
		//Point1[0] = 101;
		//Point1[1] = 260;
		//Point1[2] = -45;
	
	
		//Point2[0] = 140;
		//Point2[1] = 266;
		//Point2[2] = -47;
	
		//Point3[0] = 155;
		//Point3[1] = 242;
		//Point3[2] = -111;
	
	
		//SelectPoint1[0] = -74.43;
		//SelectPoint1[1] = 6.71;
		//SelectPoint1[2] = 9.78;
		//SelectPoint2[0] = -78.13;
		//SelectPoint2[1] = 1.94;
		//SelectPoint2[2] = 49.96;
		//SelectPoint3[0] = -145.00;
		//SelectPoint3[1] = -6.0;
		//SelectPoint3[2] = 61.0;
		for (int i = 0; i < 3; i++)
		{
			R1[i] = Point2[i] - Point1[i];
			R2[i] = Point3[i] - Point2[i];
	
			V1[i] = SelectPoint2[i] - SelectPoint1[i];
			V2[i] = SelectPoint3[i] - SelectPoint2[i];
		}
		double Mpt[3][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };//骨头上点在tracker系姿态
	    double Mps [3][3]= { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };//骨头上点在骨头坐标系下姿态
	//	double Attitude[4]  = { 1.0, 0.0, 0.0 ,0.0};


		//V1[0] = 1;
		//V1[1] = 0;
		//V1[2] = 0;

		//V2[0] = 0;
		//V2[1] = 1;
		//V2[2] = 0;

		//R1[0] = 1;
		//R1[1] = 0;
		//R1[2] = 0;

		//R2[0] = 0;
		//R2[1] = 1;
		//R2[2] = 0;
		double	Attitude[4] = { 0.592,-0.716,-0.099,-0.357};
		GetTransformMatrix(1,SelectPoint2,Point2,Attitude,V1,V2,R1,R2,Mpt,Mps);

		double Point1_osg[3] = { 0.0, 0.0, 0.0 };
		double Point2_osg[3]  = { 0.0, 0.0, 0.0 };
		double Point3_osg [3] = { 0.0, 0.0, 0.0 };

		double SelectPoint1_bone[3] = { 0.0, 0.0, 0.0 };
		double SelectPoint2_bone[3]  = { 0.0, 0.0, 0.0 };
		double SelectPoint3_bone[3]  = { 0.0, 0.0, 0.0 };


		Point1_osg[0] = 353;
		Point1_osg[1] = 266;
		Point1_osg[2] = -56;


		Point2_osg[0] = 393;
		Point2_osg[1] = 265;
		Point2_osg[2] = -62;

		Point3_osg[0] = 384;
		Point3_osg[1] = 231;
		Point3_osg[2] = -133;



		SelectPoint1_bone[0] = -71;
		SelectPoint1_bone[1] = -1;
		SelectPoint1_bone[2] = 64;
		SelectPoint2_bone[0] = -67;
		SelectPoint2_bone[1] = -3;
		SelectPoint2_bone[2] = 24;
		SelectPoint3_bone[0] = -145;
		SelectPoint3_bone[1] = 7;
		SelectPoint3_bone[2] = 15;


		/*Point1_osg[0] = 353;
		Point1_osg[1] = 266;
		Point1_osg[2] = -56;


		Point2_osg[0] = 393;
		Point2_osg[1] = 265;
		Point2_osg[2] = -62;

		Point3_osg[0] = 384;
		Point3_osg[1] = 231;
		Point3_osg[2] = -133;



		SelectPoint1_bone[0] = -71.13;
		SelectPoint1_bone[1] = 6.39;
		SelectPoint1_bone[2] = 12.73;
		SelectPoint2_bone[0] = -68.21;
		SelectPoint2_bone[1] = 7.17;
		SelectPoint2_bone[2] = 53.26;
		SelectPoint3_bone[0] = -146.01;
		SelectPoint3_bone[1] = 15.49;
		SelectPoint3_bone[2] = 62.57;*/
		double tracker1 [3]= { 0.0, 0.0, 0.0 };
		double tracker2 [3]= { 0.0, 0.0, 0.0 };
		double osg1[3] = { 0.0, 0.0, 0.0 };
		double osg2[3]= { 0.0, 0.0, 0.0 };

		for (int i = 0; i < 3; i++)
		{
			tracker1[i] = Point2_osg[i] - Point1_osg[i];
			tracker2[i] = Point3_osg[i] - Point2_osg[i];

			osg1[i] = SelectPoint2_bone[i] - SelectPoint1_bone[i];
			osg2[i] = SelectPoint3_bone[i] - SelectPoint2_bone[i];
		}
		//for (int i = 0; i < 3; i++)
		//{
		//	R1[i] = Point2_osg[i] - Point1_osg[i];
		//	R2[i] = Point3_osg[i] - Point2_osg[i];

		//	V1[i] = SelectPoint2_bone[i] - SelectPoint1_bone[i];
		//	V2[i] = SelectPoint3_bone[i] - SelectPoint2_bone[i];
		//}

		//osg1[0] = 1;
		//osg1[1] = 0;
		//osg1[2] = 0;

		//osg2[0] = 0;
		//osg2[1] = 1;
		//osg2[2] = 0;

		//Attitude[0] = 90;
		//Attitude[1] = -50;
		//Attitude[2] = 32; 
	double	Attitude2[4] = { 0.623,0.659,-0.035,0.420};
		GetTransformMatrix(0,SelectPoint1_bone,Point1_osg,Attitude2,tracker1, tracker2, osg1, osg2, Mps, Mpt);


	DWORD dwThreadID;
	HANDLE hThread;

	hThread = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)action,0,0,&dwThreadID);
	Sleep(100);
	if ( ! SetThreadPriority(hThread,THREAD_PRIORITY_ABOVE_NORMAL) ) return 0;
	
    //viewer->setSceneData(root.get());
    //viewer->realize();
    //viewer->run() ;  

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
	win->getViewer()->setSceneData(root.get());
	win->getViewer()->setCameraManipulator(new osgGA::TrackballManipulator);

	while(!win->isQuit())
	{
		win->draw();
	}
	delete win;
	return 0;
}
void EndDisplay()
{
/*	viewer->setSceneData(NULL);*/
	viewer->stopThreading();
	viewer->setDone(true);

	viewer->setQuitEventSetsDone(false);
	viewer->setKeyEventSetsDone(0x20);
}