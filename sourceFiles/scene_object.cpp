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
                Point3D point = worldToModel * ray.intersection.point;
                ray.intersection.textcoor = Point3D(point[0] + 0.5, 0.5 - point[1], 0);
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
    bool insideCrash=false;
    // std::cout << test <<std::endl;
    if(test < 0 || a == 0){
        return false;
    }else if(test == 0){
        t = -b/(2*a);
    }else{
        double t1 = (-b + sqrt(test))/(2*a);
        double t2 = (-b - sqrt(test))/(2*a);
        if(t1>0 && t2<0){
            insideCrash=true;
            t = t1;
        }else if(t1<=0 && t2>=0){
            insideCrash=true;
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
        ray.intersection.insideCrash = insideCrash;
        Point3D p_n = Point3D(inter_x, inter_y, inter_z);
        //Vector3D n = Point3D() - p_n ;
        Vector3D n = p_n - Point3D();
        n.normalize();
        Vector3D inter_norm = transNorm(worldToModel, n);
        ray.intersection.normal = inter_norm;
       
        Point3D point = worldToModel * ray.intersection.point;
        double s = acos(point[2]/1.0)/M_PI;
        double t = acos(point[0]/(1.0*sin(M_PI * s)))/(2.0*M_PI);
        ray.intersection.textcoor = Point3D(s, t, 0);
        
        return true;
    }else{
        return false;
    }
    
}

//bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
//		const Matrix4x4& modelToWorld ) {
//	// TODO: implement intersection code for UnitSquare, which is
//	// defined on the xy-plane, with vertices (0.5, 0.5, 0), 
//	// (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and normal
//	// (0, 0, 1).
//	//
//	// Your goal here is to fill ray.intersection with correct values
//	// should an intersection occur.  This includes intersection.point, 
//	// intersection.normal, intersection.none, intersection.t_value.   
//	//
//	// HINT: Remember to first transform the ray into object space  
//	// to simplify the intersection test.
//	
//    //transform the ray into object space
//    ray.origin = worldToModel*ray.origin;
//    ray.dir = worldToModel*ray.dir;
//    
//    Point3D plane_origin;
//    Vector3D plane_normal(0,0,1);
//    if(plane_normal.dot(ray.dir)==0){
//        //transform back to world space;
//        ray.origin = modelToWorld*ray.origin;
//        ray.dir = modelToWorld*ray.dir;
//        
//        return false;
//    }else{
//        
//        double t =(plane_normal.dot(plane_origin-ray.origin))/(plane_normal.dot(ray.dir));
//        if(t<=0){
//           
//            //transform back to world space;
//            ray.origin = modelToWorld*ray.origin;
//            ray.dir = modelToWorld*ray.dir;
//            return false;
//        }else{
//        double inter_x=ray.origin[0]+t*ray.dir[0];
//        double inter_y=ray.origin[1]+t*ray.dir[1];
//            
//            if(inter_x < 0.5 && inter_x > -0.5 && inter_y < 0.5 && inter_y > -0.5 && (t < ray.intersection.t_value || ray.intersection.none)){
//                ray.intersection.point = modelToWorld * Point3D(inter_x, inter_y, 0);
//                ray.intersection.t_value = t;
//                ray.intersection.none = false;
//                ray.intersection.normal= transNorm(worldToModel, plane_normal);
//                ray.intersection.normal.normalize();
//                //transform back to world space;
//                ray.origin = modelToWorld*ray.origin;
//                ray.dir = modelToWorld*ray.dir;
//                return true;
//            }else{
//               // ray.intersection.none=true;
//                //transform back to world space;
//                ray.origin = modelToWorld*ray.origin;
//                ray.dir = modelToWorld*ray.dir;
//                return false;
//            }
//        }
//    
//    }
//    
//	
//}

bool UnitFiniteCylinder::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
                                     const Matrix4x4& modelToWorld ) {
    Point3D ray_origin = worldToModel*ray.origin;
    Vector3D ray_dir = worldToModel*ray.dir;
    Vector3D va(0,1,0);//axis for cylinder
    Point3D pa(0,0,0);
    Point3D pb(0, 1, 0);
    bool insideFlag1=false,insideFlag2=false;
   //At^2+B*t+C-1=0
    double vva=ray_dir.dot(va);//the projection
    double A=(ray_dir-vva*va).dot(ray_dir-vva*va);
    double B = 2*(ray_dir-vva*va).dot(((ray_origin-pa)-(ray_origin-pa).dot(va)*va));
    double C=(ray_origin-pa-(ray_origin-pa).dot(va)*va).dot(ray_origin-pa-vva*va)-1;
    double t=0;
    double test = B*B-4*A*C;
    if(test<0||A==0){ return false;//no intersection
    }else if(test==0){
        t=-B/(2*A);
        
    }else{//there are two roots.
        //testing for the infinite cylinder
       
        
        double t1 = (-B+ sqrt(test))/(2*A);
        double t2 = (-B - sqrt(test))/(2*A);
        if(t1>0 && t2<0){
            t = t1;
            insideFlag1=true;
        }else if(t1<0 && t2>0){
            t = t2;
            insideFlag1 = true;
        }else if(t1 > 0 && t2 > 0){
            if(t1 > t2){
                t = t2;
            }else{
                t = t1;
            }
        }else{//cylinder is behind the ray_origin so no intersection
            return false;
        }
        
        //testing for between the two planes.
        
        Point3D inter_pt_tmp = ray_origin+t*ray_dir;
        if(((inter_pt_tmp-pa).dot(va)>0)&&((inter_pt_tmp-pb).dot(va)<0)){//between the two caps
        }else{
            t=std::numeric_limits<double>::max();
        }

        
        //testing for caps
        double t3 = (pa-ray_origin).dot(va)/ray_dir.dot(va);//intersection with the lower cap
        double t4 = (pb-ray_origin).dot(va)/ray_dir.dot(va);//intersection with the upper cap
        double tt=0;
        Point3D po;
        if(t3<0&&t4<0){
            return false;
        }else if(t3>0&&t4<0){
            tt=t3;
            po=pa;
            insideFlag2 = true;
        }else if(t3<0&&t4>0){
            tt=t4;
            po=pb;
            insideFlag2 = true;
        }else{
            if(t4 > t3){
                tt=t3;
                po=pa;
            }else{
                tt=t4;
                po=pb;
            }
        }
        
        Point3D inter_pt_tmp1 = ray_origin+tt*ray_dir;
       if((inter_pt_tmp1-po).length()<1){//in the cylinder

        }else{//must on the cylinders
            tt=std::numeric_limits<double>::max();
        }
        
        if(t<=tt){
        if(t==std::numeric_limits<double>::max()) return false;
        if(ray.intersection.none||t<ray.intersection.t_value){
            inter_pt_tmp = ray_origin+t*ray_dir;
            ray.intersection.point = modelToWorld*inter_pt_tmp;
            
            Vector3D cylinder_norm;
            if(insideFlag1){
                 cylinder_norm =Point3D(0,inter_pt_tmp[1],0)-inter_pt_tmp;
            }else{
                 cylinder_norm =inter_pt_tmp-Point3D(0,inter_pt_tmp[1],0);
            }
            cylinder_norm.normalize();
            ray.intersection.normal = transNorm(modelToWorld,cylinder_norm );
            ray.intersection.normal.normalize();
            ray.intersection.t_value=t;
            ray.intersection.none=false;
            ray.intersection.insideCrash = insideFlag1;
            return true;
        }else{
            return true;
        }
        }else{
            if(ray.intersection.none||tt<ray.intersection.t_value){
                inter_pt_tmp = ray_origin+tt*ray_dir;
                ray.intersection.point = modelToWorld*inter_pt_tmp;
//                if(tt==t3){
//                    ray.intersection.normal = transNorm(modelToWorld, -va);
//                }else{
//                ray.intersection.normal = transNorm(modelToWorld, va);
//                }
                                if((tt==t3&&insideFlag1==false)||(tt==t4&&insideFlag1)){
                                    ray.intersection.normal = transNorm(modelToWorld, -va);
                                }else if((tt==t4&&insideFlag1==false)||(tt==t3&&insideFlag1)){
                                ray.intersection.normal = transNorm(modelToWorld, va);
                                }

                ray.intersection.normal.normalize();
                ray.intersection.t_value=tt;
                ray.intersection.none=false;
                ray.intersection.insideCrash = insideFlag2;
                return true;
            }else{
                return true;
            }
        }
    }
    return false;
    }


bool UnitFiniteCone::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
                                   const Matrix4x4& modelToWorld ) {
    bool insideFlag1=false,insideFlag2=false;
    
    Point3D ray_origin = worldToModel*ray.origin;
    Vector3D ray_dir = worldToModel*ray.dir;
    Vector3D va(0,1,0);//axis for cylinder
    Point3D pa(0,0,0);
    Point3D pb(0, -1, 0);//the apex of the cone
    double cos_sqr= 0.5;
    double sin_sqr = 0.5;
    Point3D inter_pt_tmp;
    //At^2+B*t+C-1=0
   
    
    
    double vva=ray_dir.dot(va);//the projection
    double A=cos_sqr*(ray_dir-vva*va).dot(ray_dir-vva*va)-sin_sqr*vva*vva;
    double B = 2*cos_sqr*(ray_dir-vva*va).dot((ray_origin-pa)-(ray_origin-pa).dot(va)*va)-2*sin_sqr*vva*(ray_origin-pa).dot(va);
    double C=cos_sqr*((ray_origin-pa)-(ray_origin-pa).dot(va)*va).dot((ray_origin-pa)-(ray_origin-pa).dot(va)*va)-sin_sqr*((ray_origin-pa).dot(va))*((ray_origin-pa).dot(va));
    double t=0;
    double test = B*B-4*A*C;
    if(test<0||A==0){ return false;//no intersection
    }else if(test==0){
        t=-B/(2*A);
        
    }else{//there are two roots.
        //testing for the infinite cylinder
        
        double t1 = (-B+ sqrt(test))/(2*A);
        double t2 = (-B - sqrt(test))/(2*A);
        if(t1>0 && t2<0){
            t = t1;
            insideFlag1=true;
        }else if(t1<0 && t2>0){
            t = t2;
            insideFlag1=true;
        }else if(t1 > 0 && t2 > 0){
            if(t1 > t2){
                t = t2;
            }else{
                t = t1;
            }
        }else{//cylinder is behind the ray_origin so no intersection
            return false;
        }
    }
        inter_pt_tmp = ray_origin+t*ray_dir;
        
        if(((inter_pt_tmp-pb).dot(va)>=0)&&((inter_pt_tmp-pa).dot(va)<=0)){//between the two caps
            
        }else{
            t=std::numeric_limits<double>::max();
        }
        
    
    //testing for caps
  
    double t3 = (pb-ray_origin).dot(va)/ray_dir.dot(va);//intersection with the lower cap
    double tt=0;
   
    if(t3<0){
        return false;
    }else{
        tt=t3;
    }
    
    Point3D inter_pt_tmp1 = ray_origin+tt*ray_dir;
    if((inter_pt_tmp1-pb).length()<1){//in the cylinder
        insideFlag2=true;
    }else{//must on the cylinders
        tt=std::numeric_limits<double>::max();
    }
    
    if(t<=tt){
     
        if(t==std::numeric_limits<double>::max()) return false;
        if(ray.intersection.none||t<ray.intersection.t_value){
           // std::cout<<"Here......1.."<<std::endl;
        inter_pt_tmp = ray_origin+t*ray_dir;
        ray.intersection.point = modelToWorld*inter_pt_tmp;
            Vector3D v;
            Vector3D cone_norm;
            if(insideFlag1){
                v =Point3D(0,inter_pt_tmp[1],0)-inter_pt_tmp;

                cone_norm = v+Vector3D(0, inter_pt_tmp[1], 0);
            }else{
                v =inter_pt_tmp-Point3D(0,inter_pt_tmp[1],0);
                cone_norm = v-Vector3D(0, inter_pt_tmp[1], 0);
            }
            cone_norm.normalize();
        ray.intersection.normal = transNorm(modelToWorld, cone_norm);
        ray.intersection.normal.normalize();
        ray.intersection.t_value=t;
        ray.intersection.none=false;
        ray.intersection.insideCrash = insideFlag1;
        return true;
        }else{
            return true;
        }
    }else{
      //  std::cout<<"Here......2.."<<std::endl;
        if(ray.intersection.none||tt<ray.intersection.t_value){
            
            inter_pt_tmp = ray_origin+tt*ray_dir;
            ray.intersection.point = modelToWorld*inter_pt_tmp;
            if(insideFlag2){
                ray.intersection.normal = transNorm(modelToWorld, va);
            }else{
            ray.intersection.normal = transNorm(modelToWorld, -va);
            }
            ray.intersection.normal.normalize();
            ray.intersection.t_value=tt;
            ray.intersection.none=false;
            ray.intersection.insideCrash = insideFlag2;
            return true;
        }else{
            return true;
        }
    }

    return false;

}
