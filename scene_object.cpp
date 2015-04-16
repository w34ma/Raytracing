/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "scene_object.h"

bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	
    //tranform ray into object space
    Vector3D obj_dir = worldToModel * ray.dir;
    Point3D obj_ori = worldToModel * ray.origin;
    //ray =obj_ori + t*obj_dir
    // N·(E + tD - Q) = 0
    // t = N·(Q - E) / N·D
    Vector3D plane_norm = Vector3D(0,0,1);
    double result = plane_norm.dot(obj_dir);
    if(result == 0){
        return false;
    }else{
        Point3D plane_ptr = Point3D();//origin point is on the plane
        Vector3D m = plane_ptr - obj_ori;
        double t = plane_norm.dot(m)/result;
        if(t > 0){
            double inter_x = obj_ori[0] + t * obj_dir[0];
            double inter_y = obj_ori[1] + t * obj_dir[1];
            if(inter_x < 0.5 && inter_x > -0.5 && inter_y < 0.5 && inter_y > -0.5 && (t < ray.intersection.t_value || ray.intersection.none)){
                ray.intersection.point = modelToWorld * Point3D(inter_x, inter_y, 0);
                ray.intersection.t_value = t;
                ray.intersection.none = false;
                Vector3D n =  Vector3D(0, 0, 1);
                Vector3D inter_norm = transNorm(worldToModel, n);
                ray.intersection.normal = inter_norm;
                ray.intersection.normal.normalize();
                return true;
            }else{
                return false;
            }
        }else{
            //no intersection
            return false;
        }
    }
}

bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
    Vector3D obj_dir = worldToModel * ray.dir;
    Point3D obj_ori = worldToModel * ray.origin;
    //ray =obj_ori + t * obj_dir
    //t^2 * (obj_dir·obj_dir)+ t * (2obj_ori·obj_dir) + obj_ori·obj_ori-1
    double a = obj_dir.dot(obj_dir);
    Vector3D trans = obj_ori - Point3D(); //cast Point3D to Vector3D to use dot
    double b = 2 * trans.dot(obj_dir);
    double c = trans.dot(trans) - 1;
    double test = b*b - 4 * a * c;
    double t = 0;

   // std::cout << test <<std::endl;
    if(test < 0 || a == 0){
        return false;
    }else if(test == 0){
         t = -b/(2*a);
    }else{
        double t1 = (-b + sqrt(test))/(2*a);
        double t2 = (-b - sqrt(test))/(2*a);
        if(t1>0 && t2<0){
             t = t1;
        }else if(t1<0 && t2>0){
             t = t2;
        }else if(t1 > 0 && t2 > 0){
            if(t1 > t2){
                 t = t2;
            }else{
                 t = t1;
            }
        }else{
            return false;
        }
    }
    double inter_x = obj_ori[0] + t * obj_dir[0];
    double inter_y = obj_ori[1] + t * obj_dir[1];
    double inter_z = obj_ori[2] + t * obj_dir[2];

    if(t > 0 && (t < ray.intersection.t_value || ray.intersection.none)){
        ray.intersection.point = modelToWorld * Point3D(inter_x, inter_y, inter_z);
        ray.intersection.t_value = t;
        ray.intersection.none = false;

        Point3D p_n = Point3D(inter_x, inter_y, inter_z);
        //Vector3D n = Point3D() - p_n ;
        Vector3D n = p_n - Point3D();
        n.normalize();
        Vector3D inter_norm = transNorm(worldToModel, n);
        ray.intersection.normal = inter_norm;

        return true;
    }
	return true;
}
