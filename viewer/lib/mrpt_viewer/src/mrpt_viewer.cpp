#include "mrpt_viewer.h"

MrptViewer::MrptViewer()
{
	win3D.setWindowTitle("CyberTiggo_View");
    win3D.resize(600, 600);
    win3D.setCameraAzimuthDeg(270);//方向沿y轴由负方向向正方向看
    win3D.setCameraElevationDeg(60);//俯角20°
    win3D.setCameraPointingToPoint(0, 10, 0);//指向(0,10,0)点
    win3D.setCameraZoom(100);

}

MrptViewer::~MrptViewer()
{

}

void MrptViewer::display(mrpt::maps::CColouredPointsMap cloud_in)
{
	mrpt::opengl::COpenGLScenePtr	scene;
	mrpt::opengl::CGridPlaneXYPtr groundPlane;
	mrpt::opengl::CAxisPtr  axis;

    groundPlane = mrpt::opengl::CGridPlaneXY::Create(-300,300,-300,300, 0,5); //地平线网格间隔
    groundPlane->setColor(0.0f, 0.0f, 0.0f);
    groundPlane->setLineWidth(0.5);

    axis = mrpt::opengl::CAxis::Create(0,0,0,2,4,2,0.25,3,false);
    axis->setColor(mrpt::utils::TColorf(0,0,1,1));

    scene = mrpt::opengl::COpenGLScene::Create();
    scene->insert(axis);
    scene->insert(groundPlane);

    mrpt::opengl::CPointCloudPtr cloud_to_view = opengl::CPointCloud::Create();
	cloud_to_view->loadFromPointsMap(&cloud_in);
	scene->insert(cloud_to_view);

    mrpt::opengl::COpenGLScenePtr &ptrScene = win3D.get3DSceneAndLock();
    ptrScene = scene;
    win3D.unlockAccess3DScene();
    win3D.forceRepaint();

}
