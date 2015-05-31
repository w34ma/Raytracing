/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		Implementations of functions in raytracer.h, 
		and the main function which specifies the 
		scene to be rendered.	

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream>
#include <cstdlib>

Raytracer::Raytracer() : _lightSource(NULL) {
	_root = new SceneDagNode();
}

Raytracer::~Raytracer() {
	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material* mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
	LightListNode* tmp = _lightSource;
	_lightSource = new LightListNode( light, tmp );
	return _lightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

// Constructs a view to world transformation matrix based on the
// camera parameters.
Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}
// Traversal code for the scene graph, the ray is transformed into
// the object space of each node where intersection is performed.


/*************************Original traverseScene**************
void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
    SceneDagNode *childPtr;
    
    // Applies transformation of the current node to the global
    // transformation matrices.
    _modelToWorld = _modelToWorld*node->trans;
    _worldToModel = node->invtrans*_worldToModel;
    if (node->obj) {
        // Perform intersection.
        if (node->obj->intersect(ray, _worldToModel, _modelToWorld)) {
            ray.intersection.mat = node->mat;
        }
    }
    // Traverse the children.
    childPtr = node->child;
    while (childPtr != NULL) {
        traverseScene(childPtr, ray);
        childPtr = childPtr->next;
    }
    
    // Removes transformation of the current node from the global
    // transformation matrices.
    _worldToModel = node->trans*_worldToModel;
    _modelToWorld = _modelToWorld*node->invtrans;
}
**********************Original traverseScene******************/

/************************Update for SCF*********************/
void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
	SceneDagNode *childPtr;
    
    
	// Applies transformation of the current node to the global
	// transformation matrices.

	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, node->worldToModel, node->modelToWorld)) {
			ray.intersection.mat = node->mat;
		}
	}
    
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		//traverseScene(childPtr, ray);
        
        if (childPtr->obj) {
            // Perform intersection.
            if (childPtr->obj->intersect(ray, childPtr->worldToModel, childPtr->modelToWorld)) {
                ray.intersection.mat = childPtr->mat;
            }
        }
        
		childPtr = childPtr->next;
        }
}

void Raytracer::scfCache(SceneDagNode *node){
    
    
    SceneDagNode *childPtr;
    
    _modelToWorld = _modelToWorld*node->trans;
    _worldToModel = node->invtrans*_worldToModel;
   if (node->obj) {
    node->modelToWorld = _modelToWorld;
    node->worldToModel = _worldToModel;
   }
    // Traverse the children.
    childPtr = node->child;
    while (childPtr != NULL) {
        scfCache(childPtr);
        childPtr = childPtr->next;
    }
    _worldToModel = node->trans*_worldToModel;
    _modelToWorld = _modelToWorld*node->invtrans;
}

/************************Update for SCF**********************************/
void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( const char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}


void Raytracer::computeShading( Ray3D& ray ) {//**********************************(shadow)*   computeShading
    LightListNode* curLight = _lightSource;
    for (;;) {
        if (curLight == NULL) break;
//        traverseScene(_root, ray);
//        if (!ray.intersection.none){
//            curLight->light->shade(ray,true);
//        }
        
   
        // Each lightSource provides its own shading function.
        
        // Implement shadows here if needed.
        
         /*********Point Light && HARD SHADOW***************
        Ray3D shadowray;
        shadowray.dir = curLight->light->get_position() - ray.intersection.point;
        shadowray.dir.normalize();
        shadowray.origin = ray.intersection.point + (1e-6) * shadowray.dir;
        
        traverseScene(_root, shadowray);
        curLight->light->shade(ray,!shadowray.intersection.none);
        //if intersected with nothing, nothing happend.
        if (!shadowray.intersection.none){
            double transCoe = ray.intersection.mat->transparency_coef;
            double k_reduction = 0.2;
            if(transCoe > 0.0){//intersect with transparent objects
                ray.col = ray.col*Colour(k_reduction,k_reduction,k_reduction);
            }else{
                ray.col = ray.intersection.mat->ambient;
            }
            
        }
        *********Area Light && SOFT SHADOW***************/
       
        Colour temp(0.0, 0.0, 0.0);
         double num_ray = 20;
         double avg = 1/num_ray;
         
         Vector3D shadowDir;
         for (double i = -1 ; i < 1.0;i = i + 0.10){
         shadowDir = curLight->light->get_position() - ray.intersection.point;
         shadowDir[0] += i;
         shadowDir[1] += i;
         shadowDir[2] += i;
         shadowDir.normalize();
         
         Point3D shadowOrigin = ray.intersection.point +  1e-6 * shadowDir;
         Ray3D shadowRay(shadowOrigin , shadowDir);
         traverseScene(_root, shadowRay);
         curLight->light->shade(ray,false);
         
         if (shadowRay.intersection.none){
         temp = temp + (avg * ray.col);
         }
         }
         ray.col = temp;
        /*************************************************/
        curLight = curLight->next;
    }
}


// After intersection, calculate the colour of the ray by shading it
// with all light sources in the scene.
Colour Raytracer::shadeRay( Ray3D& ray,int times_reflec,int times_refrac ) {//(reflection)*   shadeRay
	Colour col(0.0, 0.0, 0.0);
    
	traverseScene(_root, ray);
    
    //for reflextion here
    
	// Don't bother shading if the ray didn't hit 
	// anything.
//	if (!ray.intersection.none) {
//		computeShading(ray); 
//		col = ray.col;  
//	}
//    return col;
    
    /*******************Reflection&Refraction********************/
    Colour reflColour(0.0, 0.0, 0.0);
    Colour refraColour;
    if (!ray.intersection.none) {
        computeShading(ray);
        col = ray.col*Colour((1-ray.intersection.mat->transparency_coef), (1-ray.intersection.mat->transparency_coef), (1-ray.intersection.mat->transparency_coef));
        //for flection
        if(times_reflec < 3 && ray.intersection.mat->reflect_coef != 0&&ray.intersection.mat->transparency_coef==0){
            Ray3D reflect_ray;
            reflect_ray.dir = ray.dir - (2 * (ray.dir.dot(ray.intersection.normal)) * ray.intersection.normal);
            reflect_ray.dir.normalize();
            reflect_ray.origin = ray.intersection.point+(1e-6) * reflect_ray.dir;
            
            
            times_reflec = times_reflec + 1;
            //Colour refl = shadeRay(reflect_ray,times);
            reflColour = reflColour + ray.intersection.mat->reflect_coef*shadeRay(reflect_ray,times_reflec,times_reflec);
        }
        //for refraction
        if(times_refrac<3&&ray.intersection.mat->transparency_coef>0){
            Ray3D refraction_ray;
            double n;
            Vector3D N =ray.intersection.normal;
            if(ray.intersection.insideCrash){
                n =ray.intersection.mat->R_index;
                N = (-1)*N;
            }else{
                n=1/ray.intersection.mat->R_index;
                
            }
            double cosI = (-1.0)*N.dot(ray.dir);
            double cosT2 = 1.0 - (n*n)*(1.0- cosI*cosI );
            if(cosT2>0.0){//whether can have refraction or not
                
                double cosT = sqrt(cosT2);
                //  double tmp1 = sqrt(1.0-(R_index*ray.dir.dot(ray.intersection.normal)*ray.dir.dot(ray.intersection.normal)));
                refraction_ray.dir =n*ray.dir+(n*cosI-cosT)*N;
                
                refraction_ray.dir.normalize();
                //refraction_ray.dir =n*ray.dir+(n*tmp2-tmp1)*(ray.intersection.normal);
                refraction_ray.origin = ray.intersection.point+(1e-6)*refraction_ray.dir;
                times_refrac++;
                refraColour = refraColour+ray.intersection.mat->transparency_coef*shadeRay(refraction_ray,  times_refrac, times_refrac);
            }
            
            }
    }
    
    
    col = col + reflColour+refraColour;
    col.clamp();
    
    return col;
     /***************END OF Reflection&Refraction***********************/
    
    /*******************Reflection********************
    Colour reflColour(0.0, 0.0, 0.0);
     if (!ray.intersection.none) {
     computeShading(ray);
     col = ray.col;
     if(times < 3 && ray.intersection.mat->reflect_coef != 0){
     Ray3D reflect_ray;
     
     reflect_ray.dir = ray.dir - (2 * (ray.dir.dot(ray.intersection.normal)) * ray.intersection.normal);
     reflect_ray.dir.normalize();
     reflect_ray.origin = ray.intersection.point+(1e-6) * reflect_ray.dir;
     
     times = times + 1;
     //Colour refl = shadeRay(reflect_ray,times);
     reflColour = reflColour + ray.intersection.mat->reflect_coef*shadeRay(reflect_ray,times);
     }
     }
     col = col + reflColour;
     col.clamp();
     
     return col;
    ***************END OF Reflection***********************/
    /*******************Glossy********************
    Colour reflColour(0.0, 0.0, 0.0);
     
     if (!ray.intersection.none) {
     computeShading(ray);
     col = ray.col;
     if (times < 3 && ray.intersection.mat->reflect_coef != 0) {
     
     Ray3D reflect_ray;
     reflect_ray.dir = (ray.dir - ((2*(ray.dir.dot(ray.intersection.normal)))*ray.intersection.normal));
     reflect_ray.dir.normalize();
     reflect_ray.origin = ray.intersection.point + 1e-6*reflect_ray.dir;
     
     double L = 0.5; // roughness
     
     for (int i = 0; i < 4; i++) {
     //r(i) = normalize(r+(0.5-xi)Lu + (0.5-yi)Lv)
     double xi = (double)rand() / (double)RAND_MAX;
     double yi = (double)rand() / (double)RAND_MAX;
     
     reflect_ray.dir[0] += (0.5-xi)*L;
     reflect_ray.dir[1] += (0.5-yi)*L;
     
     reflect_ray.dir.normalize();
     
     times = times + 1;
     reflColour = reflColour + ray.intersection.mat->reflect_coef*shadeRay(reflect_ray , times);
     }
     reflColour = (1.0/4.0)*reflColour;
     }
     col = col + reflColour;
     col.clamp();
     }
     
     return col;
    ****************END OF GLOSSY********************************/
    
	// You'll want to call shadeRay recursively (with a different ray,
	// of course) here to implement reflection/refraction effects.  ...........................

	
}

// Renders an image fileName with width and height and a camera
// positioned at eye, with view vector view, up vector up, and
// field of view fov.

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, const char* fileName ) {
    /*****************anti-aliasing************************/
    Matrix4x4 viewToWorld;
    _scrWidth = width;
    _scrHeight = height;
    double factor = (double(height)/2)/tan(fov*M_PI/360.0);
    Colour totalCol(0.0,0.0,0.0);
    
    initPixelBuffer();
    viewToWorld = initInvViewMatrix(eye, view, up);
    scfCache(_root);
    // Construct a ray for each pixel.
    
    for (int i = 0; i < _scrHeight; i++) {
        for (int j = 0; j < _scrWidth; j++) {
            //anti-aliasing, each pixel can have 4 rays, the pixel color determined by the avgerage
            for(double a = i; a < i+1 ; a += 0.5){
                for(double b = j; b < j+1;b += 0.5){
                    
                    // Sets up ray origin and direction in view space,
                    // image plane is at z = -1.
                    Point3D origin(0, 0, 0);
                    Point3D imagePlane;
                    imagePlane[0] = (-double(width)/2 + 0.5 + b)/factor;
                    imagePlane[1] = (-double(height)/2 + 0.5 + a)/factor;
                    imagePlane[2] = -1;
                    
                    // TODO: Convert ray to world space and call
                    // shadeRay(ray) to generate pixel colour.
                    Point3D originW = viewToWorld * imagePlane;
                    Vector3D directionW = viewToWorld * (imagePlane -origin);
                    directionW.normalize();
                    
                    Ray3D ray(originW, directionW);
                    Colour col = shadeRay(ray,0,0);
                    
                    //each ray contributed 0.25 color to the final rending color for the pixel
                    _rbuffer[i*width+j] += int(col[0]*255*0.25);
                    _gbuffer[i*width+j] += int(col[1]*255*0.25);
                    _bbuffer[i*width+j] += int(col[2]*255*0.25);
                }
            }
        }
    
    }
/********************anti-aliasing**************************/
   
//	Matrix4x4 viewToWorld;
//	_scrWidth = width;
//	_scrHeight = height;
//	double factor = (double(height)/2)/tan(fov*M_PI/360.0);
//
//	initPixelBuffer();
//	viewToWorld = initInvViewMatrix(eye, view, up);
//    scfCache(_root);
//	// Construct a ray for each pixel.
//	for (int i = 0; i < _scrHeight; i++) {
//		for (int j = 0; j < _scrWidth; j++) {
//			// Sets up ray origin and direction in view space, 
//			// image plane is at z = -1.
//			Point3D origin(0, 0, 0);
//			Point3D imagePlane;
//			imagePlane[0] = (-double(width)/2 + 0.5 + j)/factor;
//			imagePlane[1] = (-double(height)/2 + 0.5 + i)/factor;
//			imagePlane[2] = -1;
//
//			// TODO: Convert ray to world space and call 
//			// shadeRay(ray) to generate pixel colour.
//            
//            //Vector3D dir = imagePlane-origin;
//            Ray3D ray(origin,imagePlane-origin);
//            /*my dode here*/
//            ray.origin = viewToWorld*ray.origin;
//            ray.dir = viewToWorld*ray.dir;
//            ray.dir.normalize();
//			Colour col = shadeRay(ray,0,0);//the original is shadeRay(ray)
//
//			_rbuffer[i*width+j] = int(col[0]*255);
//			_gbuffer[i*width+j] = int(col[1]*255);
//			_bbuffer[i*width+j] = int(col[2]*255);
//		}
//	}

	flushPixelBuffer(fileName);
}

int main(int argc, char* argv[])
{	
	// Build your scene and setup your camera here, by calling 
	// functions from Raytracer.  The code here sets up an example
	// scene and renders it from two different view points, DO NOT
	// change this if you're just implementing part one of the 
	// assignment.  
	Raytracer raytracer;
	int width = 320; 
	int height = 240; 

	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	}
    
/***********************************************************Testing ********************************
    // Camera parameters.
    Point3D eye(0, 0, 1);
    Vector3D view(0, 0, -1);
    Vector3D up(0, 1, 0);
    double fov = 60;
    
    // Defines a material for shading.
    Material gold( Colour(0.3, 0.3, 0.3), Colour(0.75164, 0.60648, 0.22648),
                  Colour(0.628281, 0.555802, 0.366065),
                  51.2,0.3,0,NULL );
    Material jade( Colour(0, 0, 0), Colour(0.54, 0.89, 0.63),
                  Colour(0.316228, 0.316228, 0.316228),
                  12.8,0.3,0,NULL);
    
    // Defines a point light source.
    raytracer.addLightSource( new PointLight(Point3D(0.0, 0, 5),
                                             Colour(0.9, 0.9, 0.9) ) );
    
    // Add a unit square into the scene with material mat.
    SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &gold );
    SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade );
    
    // Apply some transformations to the unit square.
    double factor1[3] = { 1.0, 2.0, 1.0 };
    double factor2[3] = { 6.0, 6.0, 6.0 };
    raytracer.translate(sphere, Vector3D(0, 0, -5));
    raytracer.rotate(sphere, 'x', -45);
    raytracer.rotate(sphere, 'z', 45);
    raytracer.scale(sphere, Point3D(0, 0, 0), factor1);
    
    raytracer.translate(plane, Vector3D(0, 0, -7));
    raytracer.rotate(plane, 'z', 45);
    raytracer.scale(plane, Point3D(0, 0, 0), factor2);
    
    
    // Render the scene, feel free to make the image smaller for
    // testing purposes.
    raytracer.render(width, height, eye, view, up, fov, "view4.bmp");
    
    // Render it from a different point of view.
    Point3D eye2(4, 2, 1);
    Vector3D view2(-4, -2, -6);
    raytracer.render(width, height, eye2, view2, up, fov, "view5.bmp");
***********************************************************Testing ********************************/
/***********************************************************Final Scene********************************/
    // Camera parameters.
//	Point3D eye(0, 8, -3);
//	Vector3D view(0, -1,0);
    Point3D eye(0, 0, 1);
    Vector3D view(0, 0, -1);
    
	Vector3D up(0, 1, 0);
	double fov = 60;

	// Defines a material for shading.
	Material gold( Colour(0.3, 0.3, 0.3), Colour(0.75164, 0.60648, 0.22648), 
			Colour(0.628281, 0.555802, 0.366065), 
			51.2,0.2,NULL);
//	Material jade( Colour(0, 0, 0), Colour(0.54, 0.89, 0.63), 
//			Colour(0.316228, 0.316228, 0.316228), 
//			12.8,0.5,NULL);
    Material jade( Colour(0, 0, 0), Colour(0.47, 0.576, 0.859),
                  Colour(0.316228, 0.316228, 0.316228),
                  12.8,0.5,NULL);
    Material red( Colour(0.3, 0.3, 0.3), Colour(1, 0, 0),
                 Colour(0.628281, 0.555802, 0.366065),
                 51.2,0.2,NULL);
    
    Material white( Colour(0.3, 0.3, 0.3), Colour(1, 0.8549, 0.7255),
                 Colour(0.628281, 0.555802, 0.366065),
                 51.2,0.2,NULL);
    Material pink( Colour(0.3, 0.3, 0.3), Colour(0.9412, 0.502, 0.502),
                   Colour(0.628281, 0.555802, 0.366065),
                   51.2,0.2,NULL);
    
    Material mirror( Colour(0.0, 0.0, 0.0), Colour(0.0, 0.0, 0.0),
                 Colour(0.0, 0.0, 0.0),
                 51.2,1,NULL);
    
    Material glass( Colour(0.3, 0.3, 0.3), Colour(1, 1, 1),
                    Colour(0.628281, 0.555802, 0.366065),
                    51.2,0,1,NULL);
    glass.R_index = 1.3;
    glass.transparency_coef=1;
	// Defines a point light source.
	raytracer.addLightSource( new PointLight(Point3D(0, 0, 5),
				Colour(0.9, 0.9, 0.9) ) );

    raytracer.addLightSource( new PointLight(Point3D(0, 6, -1),
                                             Colour(0.9, 0.3, 0.1) ) );
    
    Material test( Colour(0.3, 0.3, 0.3), Colour(0.3, 0.60648, 0.22648),
                  Colour(0.628281, 0.555802, 0.366065),
                  51.2 ,0.1,NULL);
    
    Material test3( Colour(0.3, 0.3, 0.3), Colour(0.3, 0.5, 0.22648),
                  Colour(0.628281, 0.555802, 0.366065),
                  51.2,1,NULL );
    
    Material test2( Colour(0, 0, 0), Colour(0.3, 0.3, 0.3),
                   Colour(1.0, 1.0, 1.0),
                   51.2,0,NULL );
    Texture sky("/Users/bingxu/Documents/graphics/COMP3271_assignment_4_template/raytracerMacOS/sky.bmp");
    Texture board("/Users/bingxu/Documents/graphics/COMP3271_assignment_4_template/raytracerMacOS/board.bmp");
    Material starrysky(Colour(0, 0, 0),Colour(0, 0, 0),
                       Colour(0.1, 0.1, 0.1), 11.264, 0, &sky);
    
    Material board_mat(Colour(0, 0, 0),Colour(0, 0, 0),
                       Colour(0.1, 0.1, 0.1), 11.264, 1, &board);
    
   
    SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade );
    SceneDagNode* plane1 = raytracer.addObject( new UnitSquare(), &jade );
    SceneDagNode* plane2 = raytracer.addObject( new UnitSquare(), &board_mat );//the bottom
   
    
    SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &mirror);
    SceneDagNode* sphere1 = raytracer.addObject( new UnitSphere(), &white );
    SceneDagNode* mars = raytracer.addObject( new UnitSphere(), &glass );
    SceneDagNode* earth = raytracer.addObject( new UnitSphere(), &pink );
    
    SceneDagNode* cylinder1 = raytracer.addObject( new UnitFiniteCylinder(), &gold );
    SceneDagNode* cylinder2 = raytracer.addObject( new UnitFiniteCylinder(), &gold );
    SceneDagNode* cylinder3 = raytracer.addObject( new UnitFiniteCylinder(), &gold );
    SceneDagNode* cone = raytracer.addObject( new UnitFiniteCone(), &red );


        double factor1[3] = { 2.0, 2.0, 2.0 };
        double factor2[3] = {50,50,50};
        double factor3[3] = { 1.0, 1.0, 1.0};
        double factor4[3] = { 1.0, 2, 1.0};
        double factor5[3] = {0.5,0.5,0.5};
        double factor6[3] = {0.5,1.5,0.5};
        double factor7[3] = {1.0,4.0,1.0};
    //3 squares
    	raytracer.translate(plane, Vector3D(0, 0, -15));
    	raytracer.scale(plane, Point3D(0, 0, 0), factor2);
    
        raytracer.translate(plane1, Vector3D(-15, 0, 0));
        raytracer.rotate(plane1, 'y', 90);
        raytracer.scale(plane1, Point3D(0, 0, 0), factor2);
   
        raytracer.translate(plane2, Vector3D(0, -8, 0));
        raytracer.rotate(plane2, 'x', -90);
        raytracer.scale(plane2, Point3D(0, 0, 0), factor2);

    //four balls
    raytracer.translate(sphere, Vector3D(-1, -6, -2));
    raytracer.scale(sphere, Point3D(0, 0, 0), factor3);
    
    
    raytracer.translate(sphere1,Vector3D(-4.5, -6, 1));
    raytracer.scale(sphere1, Point3D(0, 0, 0), factor3);
    
    raytracer.translate(mars, Vector3D(3, -3, -1));
    raytracer.scale(mars, Point3D(0, 0, 0), factor3);

   
    raytracer.translate(earth, Vector3D(-8, -6, -2));
    raytracer.scale(earth, Point3D(0, 0, 0), factor3);
    
    
    raytracer.rotate(cylinder1, 'z', -30);
    //raytracer.rotate(cylinder1, 'x', -15);
    raytracer.translate(cylinder1, Vector3D(0, -4, -2));
    raytracer.scale(cylinder1, Point3D(0, 0, 0), factor7);
   
    raytracer.rotate(cylinder2, 'z', -30);
    raytracer.translate(cylinder2, Vector3D(1.5, -3, -2));
    raytracer.scale(cylinder2, Point3D(0, 0, 0), factor6);
    
    raytracer.rotate(cylinder3, 'z', -30);
    raytracer.translate(cylinder3, Vector3D(-1.5, -3, -2));
    raytracer.scale(cylinder3, Point3D(0, 0, 0), factor6);
    
     raytracer.rotate(cone, 'z', -30);
     raytracer.translate(cone, Vector3D(0, 2, -2));
     raytracer.scale(cone, Point3D(0, 0, 0), factor4);
    
    std::clock_t start;
    double duration;
    
    start = std::clock();
   // raytracer.render(width, height, eye, view, up, fov, "view4.bmp");
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    
    //std::cout<<"The rendering duration 1 is .......: "<< duration <<'\n';
    // Render it from a different point of view.
    
    Point3D eye2(3, 1, 5);
    Vector3D view2(-10, -8, -15);

    
    std::clock_t start1;
    double duration1;
    
    start1 = std::clock();
    raytracer.render(width, height, eye2, view2, up, fov, "view5.bmp");
    duration1 = ( std::clock() - start1 ) / (double) CLOCKS_PER_SEC;
    
   // std::cout<<"The rendering duration 2 is .......: "<< duration1 <<'\n';
    
    
    /***********************************************************Final Scene********************************/

        return 0;
}

