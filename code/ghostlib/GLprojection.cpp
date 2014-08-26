
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <stdio.h>

#include <math.h>
#include <opencv2\opencv.hpp>
#include <cylinder.h>

#include "cylindermapping.h"
#include "cvutil.h"
#include "ghostcam.h"

#include "GLProjection.h"

GLuint gFBO;
GLuint gFBOTexture[1];
GLuint gFBOTextureDepth[1];


//http://sightations.wordpress.com/2010/08/03/simulating-calibrated-cameras-in-opengl/
void reshape (int w, int h) {

	glViewport (0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();

	glLoadIdentity();

	glOrtho(0,640,480,0,1,100);
	
	cv::Mat camMat_t = mat3_to_mat4(getCameraMatrix()).t();
	cv::Mat camMat2_t = camMat_t.clone();
	camMat2_t.ptr<float>(2)[0] *= -1;
	camMat2_t.ptr<float>(2)[1] *= -1;
	camMat2_t.ptr<float>(2)[2] = 1 + 100;
	camMat2_t.ptr<float>(2)[3] = -1;
	camMat2_t.ptr<float>(3)[2] = 100;
	camMat2_t.ptr<float>(3)[3] = 0;

	glMultMatrixf(camMat2_t.ptr<float>());
}


void load_texture(cv::Mat tex_mat_, GLuint * texture){
	cv::Mat tex_mat;
	cv::flip(tex_mat_, tex_mat, 0);
	glEnable( GL_TEXTURE_2D );
	
	if(tex_mat.data == NULL){
		std::cout << "failed to load texture...\n";
	}else{
		glGenTextures(1, texture);
		glBindTexture(GL_TEXTURE_2D, texture[0]);

		// select modulate to mix texture with color for shading
		glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

		// when texture area is small, bilinear filter the closest mipmap
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_LINEAR); 

		// if wrap is true, the texture wraps over at the edges (repeat)
		//       ... false, the texture ends at the edges (clamp)
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S , GL_REPEAT );
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

        glTexImage2D(GL_TEXTURE_2D, 0, 4, tex_mat.cols, tex_mat.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_mat.data);

		std::cout << "Loaded texture with ID " << texture[0] << "\n";
	}
}

void init_depth_texture(GLuint * texture, int width, int height){
	glGenTextures(1, texture);
	glBindTexture(GL_TEXTURE_2D, texture[0]);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	//NULL means reserve texture memory, but texels are undefined
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, NULL);


}

void cylinderPoints(int frame, int limbid, cv::Mat transform, cv::Mat * pts, const std::vector<Skeleton>& wcSkeletons, const CylinderBody& cylinderBody){
	
	glBindFramebuffer( GL_FRAMEBUFFER, gFBO );
	glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gFBOTexture[0], 0 );
	glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, gFBOTextureDepth[0], 0);
	glClear( GL_COLOR_BUFFER_BIT );
	//glutMainLoop();
	//display();

	glMatrixMode (GL_MODELVIEW);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_ALPHA_TEST);
	glClearColor (0.5,0.0,0.0,1.0);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();

	glLoadIdentity();
	glRotatef(180, 0, 1, 0);


	glColor3f(1,1,1);

	renderLimb(frame, limbid, transform, wcSkeletons, cylinderBody);

	glutSwapBuffers();

	cv::Mat pic(480,640,CV_8UC3);
	glReadPixels(0,0,640,480,GL_BGR,GL_UNSIGNED_BYTE,pic.ptr<unsigned char>());

	cv::Mat picflip;
	cv::flip(pic, picflip, -1);

	cv::Mat depth(480,640,CV_32F);
	glReadPixels(0,0,640,480,GL_DEPTH_COMPONENT,GL_FLOAT,depth.ptr<float>());

	cv::Mat depthflip;
	cv::flip(depth, depthflip, -1);

	depth2points(depthflip, pts);

	glBindFramebuffer( GL_FRAMEBUFFER, NULL );
}

void renderLimb(int frame, int limbid, cv::Mat transform, const std::vector<Skeleton>& wcSkeletons, const CylinderBody& cylinderBody){
	int f = getLimbmap()[limbid].first;
	int s = getLimbmap()[limbid].second;

	cv::Mat _a = wcSkeletons[frame].points.col(f);
	cv::Mat _b = wcSkeletons[frame].points.col(s);

	cv::Mat __a = _b + cylinderBody.newLeftOffset_cyl[limbid] * (_a - _b);
	cv::Mat __b = _a + cylinderBody.newRightOffset_cyl[limbid] * (_b - _a);

	cv::Vec3f a = mat_to_vec3(transform * __a);
	cv::Vec3f b = mat_to_vec3(transform * __b);

	float radius = cylinderBody.newPartRadii_cyl[limbid];

	renderCylinder_convenient(a(0), a(1), a(2), b(0), b(1), b(2), radius, radius, 16);
}


cv::Mat cylinder_to_ptsGL(cv::Vec3f a, cv::Vec3f b, float radius, cv::Point voff, std::vector<cv::Vec3f> * fromPixels, std::vector<cv::Vec3s> * fromPixels_2d_v){
	glBindFramebuffer( GL_FRAMEBUFFER, gFBO );
	glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gFBOTexture[0], 0 );
	glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, gFBOTextureDepth[0], 0);
	glClear( GL_COLOR_BUFFER_BIT );
	//glutMainLoop();
	//display();

	glMatrixMode (GL_MODELVIEW);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_ALPHA_TEST);
	glClearColor (0.5,0.0,0.0,1.0);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();

	glLoadIdentity();
	glRotatef(180, 0, 1, 0);


	glColor3f(1,1,1);

	renderCylinder_convenient(a(0), a(1), a(2), b(0), b(1), b(2), radius, radius, 16);

	glutSwapBuffers();

	cv::Mat pic(480,640,CV_8UC3);
	glReadPixels(0,0,640,480,GL_BGR,GL_UNSIGNED_BYTE,pic.ptr<unsigned char>());

	cv::Mat picflip;
	cv::flip(pic, picflip, -1);

	cv::Mat depth(480,640,CV_32F);
	glReadPixels(0,0,640,480,GL_DEPTH_COMPONENT,GL_FLOAT,depth.ptr<float>());

	cv::Mat depthflip;
	cv::flip(depth, depthflip, -1);

	cv::Mat pts;

	depth2points(depthflip, &pts);

	glBindFramebuffer( GL_FRAMEBUFFER, NULL );

	return pts;
}


void initGL(int argc, char ** argv){
	
	glutInit (&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE);
	glutInitWindowSize (1, 1);
	glutInitWindowPosition (0, 0);
	glutCreateWindow ("A basic OpenGL Window");
	//glutDisplayFunc (display);
	//glutIdleFunc (display);
	//glutReshapeFunc (reshape);
	//glutTimerFunc(25, update, 0);

	//glutKeyboardFunc(keyHandle);
	//glutMouseFunc(clickHandle);
	//glutMotionFunc(moveHandle);
	
	
    GLenum glewError = glewInit();
    if( glewError != GLEW_OK )
    {
        printf( "Error initializing GLEW! %s\n", glewGetErrorString( glewError ) );
        return;
    }

    //Make sure OpenGL 2.1 is supported
    if( !GLEW_VERSION_2_1 )
    {
        printf( "OpenGL 2.1 not supported!\n" );
        return;
    }

    glGenFramebuffers( 1, &gFBO );

	int width = 640;
	int height = 480;
	reshape(width, height);
	//calcInvProjMat(width, height);
	
	//t = 0;
	//frame = 100;
	
	load_texture(cv::Mat(height, width, CV_8UC4, cv::Scalar(255,0,255,255)), gFBOTexture);
	init_depth_texture(gFBOTextureDepth, width, height);
}