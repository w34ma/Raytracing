/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"

void PointLight::shade( Ray3D& ray,bool flag ) {
	// TODO: implement this function to fill in values for ray.col 
	// using phong shading.  Make sure your vectors are normalized, and
	// clamp colour values to 1.0.
	//
	// It is assumed at this point that the intersection information in ray 
	// is available.  So be sure that traverseScene() is called on the ray 
	// before this function.
    Material* mat = ray.intersection.mat;
    //Q1
    //ray.col = mat->diffuse;
  
    Vector3D n = ray.intersection.normal;
    n.normalize();
    Vector3D l = _pos - ray.intersection.point;
    l.normalize();
    Colour L = _col_ambient * mat->ambient;
    
    double diffuse_v = n.dot(l);
    Colour D = fmax(0,diffuse_v) * _col_diffuse * mat->diffuse;
    
    Vector3D v = -ray.dir;
    v.normalize();
    Vector3D h = 2 * (l.dot(n)) * n - l;
    h.normalize();

    Colour S = pow(fmax(0.0, v.dot(h)),mat -> specular_exp) * mat -> specular * _col_specular;
    
    //Q2
    //ray.col = ray.col +L+ D;
    //Q3
 
    ray.col = L+ D + S;
    ray.col.clamp();

}

void AreaLight::shade( Ray3D& ray ,bool flag) {

    Material* mat = ray.intersection.mat;
    
    Vector3D n = ray.intersection.normal;
    n.normalize();
    Vector3D l = _pos - ray.intersection.point;
    l.normalize();
    Colour L = _col_ambient * mat->ambient;
    
    double diffuse_v = n.dot(l);
    Colour D = fmax(0,diffuse_v) * _col_diffuse * mat->diffuse;
    
    Vector3D v = -ray.dir;
    v.normalize();
    Vector3D h = 2 * (l.dot(n)) * n - l;
    h.normalize();
    
    Colour S = pow(fmax(0.0, v.dot(h)),mat -> specular_exp) * mat -> specular * _col_specular;

    ray.col = L+ D + S;
    ray.col.clamp();
}

