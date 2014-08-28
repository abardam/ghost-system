#pragma once

#include <GL/glew.h>
#include <GL/gl.h>
#include <opencv2\opencv.hpp>

void load_texture(cv::Mat tex_mat_, GLuint * texture);
void init_depth_texture(GLuint * texture, int width, int height);