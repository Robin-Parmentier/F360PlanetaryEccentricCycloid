
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#include <Core/Application/Application.h>
#include <Fusion/Fusion/Design.h>
#include <Fusion/Construction/ConstructionPlane.h>
#include <Fusion/Sketch/Sketches.h>
#include <Fusion/Sketch/Sketch.h>
#include <Fusion/Sketch/SketchCurves.h>
#include <Fusion/Sketch/SketchLines.h>
#include <Fusion/Sketch/SketchLine.h>
#include <Fusion/Sketch/Profiles.h>
#include <Fusion/Sketch/Profile.h>
#include <Fusion/Features/Features.h>
#include <Fusion/Features/ExtrudeFeatureInput.h>
#include <Fusion/Features/ExtrudeFeatures.h>
#include <Fusion/Features/ExtrudeFeature.h>
#include <Core/Application/Documents.h>
#include <Core/Application/Document.h>
#include <Core/Application/Product.h>
#include <Core/Geometry/Matrix3D.h>
#include <Core/UserInterface/UserInterface.h>
#include <Core/UserInterface/Palettes.h>
#include <Core/UserInterface/TextCommandPalette.h>
#include <Fusion/Components/Component.h>
#include <Fusion/Components/Occurrence.h>
#include <Fusion/Components/Occurrences.h>
#include <Fusion/Fusion/Design.h>

#include <cmath>

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;
using namespace std; //makes it easy to use math operators

Ptr<Application> app;
Ptr<UserInterface> ui;

//user-defined values
double planetOrbitRadius = 1.9; //distance from the center that the planets orbit around
double planetRadius = 0.5;
int number_of_planets = 10;
double contraction = 0.05; //contraction of sun and frame gear
double planet_shaft_radius = 0.2;
double step_size = 1; //determines the resolution of the points generated for the sun and frame gears

//calculated values
int sunReductionRatio = number_of_planets - 1;
int frameReductionRatio = number_of_planets + 1;
double default_eccentricity = planetOrbitRadius/ number_of_planets; //planet gear eccentricity with no contraction applied
double sun_root_radius = sunReductionRatio * default_eccentricity; //this can be thought of as the circle that our imaginary eccentricity circle rolls along to make our profile
double frame_root_radius = frameReductionRatio * default_eccentricity;
double trueEccentricity = default_eccentricity - contraction; //the real eccentricity of the planet gear after our user-defined contraction is taken into account

//convert degrees to radians
double degreesToRadians(double angle){
    return angle*(3.141592653589793238463/180.0);
}

//Given an angle, find the x and y coordinate of the sun curve profile at that angle
void calculateSunCurve(double angle, double &point_x, double &point_y) {
	double x = (sun_root_radius + default_eccentricity) * cos(degreesToRadians(angle));
	double y = (sun_root_radius + default_eccentricity) * sin(degreesToRadians(angle));

	point_x = x + (default_eccentricity - contraction) * cos(number_of_planets * degreesToRadians(angle));
	point_y = y + (default_eccentricity - contraction) * sin(number_of_planets * degreesToRadians(angle));
}

//Given an angle, find the x and y coordinate of the frame curve profile at that angle
void calculateFrameCurve(double angle, double& point_x, double& point_y) {
	double x = (frame_root_radius - default_eccentricity) * cos(degreesToRadians(angle));
	double y = (frame_root_radius - default_eccentricity) * sin(degreesToRadians(angle));

	point_x = x + (default_eccentricity - contraction) * cos(number_of_planets * degreesToRadians(-angle));
	point_y = y + (default_eccentricity - contraction) * sin(number_of_planets * degreesToRadians(-angle));
}

extern "C" XI_EXPORT bool run(const char* context)
{
	app = Application::get();
	if (!app)
		return false;

	//currently just used for debugging messages
	ui = app->userInterface();
	if (!ui)
		return false;

	Ptr<Design> design = app->activeProduct();
	if (!design)
		ui->messageBox("Failed to get design!");
		return false;

	Ptr<Component> root_component = design->rootComponent();
	if (!root_component)
		ui->messageBox("Failed to get root component!");
		return false;

	//make sun gear component
	Ptr<Occurrence> sun_occ = root_component->occurrences()->addNewComponent(adsk::core::Matrix3D::create());
	if (!sun_occ)
		ui->messageBox("Failed to get sun occurence!");
		return false;
	Ptr<Component> sun_gear = sun_occ->component();
	if (!sun_gear)
		ui->messageBox("Failed to make sun gear component!");
		return false;

	//make planet gear component
	Ptr<Occurrence> planet_occ = root_component->occurrences()->addNewComponent(adsk::core::Matrix3D::create());
	if (!planet_occ)
		ui->messageBox("Failed to make planet occurence!");
		return false;
	Ptr<Component> planet_gear = planet_occ->component();
	if (!planet_gear)
		ui->messageBox("Failed to make planet occurence!");
		return false;

	//make frame component
	Ptr<Occurrence> frame_occ = root_component->occurrences()->addNewComponent(adsk::core::Matrix3D::create());
	if (!frame_occ)
		return false;
	Ptr<Component> frame = frame_occ->component();
	if (!frame)
		return false;

	//name all of the components
	sun_gear->name("sun gear");
	planet_gear->name("planet gear");
	frame->name("frame");

	// Create an object collection for the points.
	Ptr<ObjectCollection> points = ObjectCollection::create();
	if (!points)
		return false;

	// Create a new sketch on the xy plane of the sun gear component.
	Ptr<Sketches> sun_gear_sketches = sun_gear->sketches();
	if (!sun_gear_sketches)
		return false;
	Ptr<ConstructionPlane> sun_gear_xy_plane = sun_gear->xYConstructionPlane();
	if (!sun_gear_xy_plane)
		return false;
	Ptr<Sketch> sun_sketch = sun_gear_sketches->add(sun_gear_xy_plane);
	if (!sun_sketch)
		return false;

	// Create a new sketch on the xy plane of the frame gear component.
	Ptr<Sketches> frame_sketches = frame->sketches();
	if (!frame_sketches)
		return false;
	Ptr<ConstructionPlane> frame_xy_plane = frame->xYConstructionPlane();
	if (!frame_xy_plane)
		return false;
	Ptr<Sketch> frame_sketch = frame_sketches->add(frame_xy_plane);
	if (!frame_sketch)
		return false;

	// Create a new sketch on the xy plane of the planet gear component.
	Ptr<Sketches> planet_gear_sketches = planet_gear->sketches();
	if (!planet_gear_sketches)
		return false;
	Ptr<ConstructionPlane> planet_gear_xy_plane = planet_gear->xYConstructionPlane();
	if (!planet_gear_xy_plane)
		return false;
	Ptr<Sketch> planet_gear_sketch = planet_gear_sketches->add(planet_gear_xy_plane);
	if (!planet_gear_sketch)
		return false;

	// Create an object collection for the points.
	Ptr<ObjectCollection> sun_points = ObjectCollection::create();
	if (!sun_points)
		return false;

	// Create an object collection for the points.
	Ptr<ObjectCollection> frame_points = ObjectCollection::create();
	if (!frame_points)
		return false;

	double point_x{ 0 };
	double point_y{ 0 };
	//find the spline points for the sun and frame gears using our step size
	for (double angle{ 0.0 }; angle <= 360.0; angle = angle + step_size) {
		calculateSunCurve(angle, point_x, point_y);
		sun_points->add(Point3D::create(point_x, point_y, 0));

		calculateFrameCurve(angle, point_x, point_y);
		frame_points->add(Point3D::create(point_x, point_y, 0));
	}

	// Create the sun spline and curves.
	Ptr<SketchFittedSpline> sun_spline = sun_sketch->sketchCurves()->sketchFittedSplines()->add(sun_points);
	if (!sun_spline)
		return false;
	Ptr<ObjectCollection> sun_curves = sun_sketch->findConnectedCurves(sun_spline);
	if (!sun_curves)
		return false;
	Ptr<ObjectCollection> sun_offset_curves = sun_sketch->offset(sun_curves, Point3D::create(0, 0, 0), planetRadius);
	if (!sun_offset_curves)
		return false;

	// create the frame spline and curves
	Ptr<SketchFittedSpline> frame_spline = frame_sketch->sketchCurves()->sketchFittedSplines()->add(frame_points);
	if (!frame_spline)
		return false;
	Ptr<ObjectCollection> frame_curves = frame_sketch->findConnectedCurves(frame_spline);
	if (!frame_curves)
		return false;
	Ptr<ObjectCollection> frame_offset_curves = frame_sketch->offset(frame_curves, Point3D::create(0, 0, 0), -planetRadius);
	if (!frame_offset_curves)
		return false;

	//clean original curves up
	design->deleteEntities(frame_curves);
	design->deleteEntities(sun_curves);
	design->deleteEntities(frame_spline);
	design->deleteEntities(sun_spline);

	//add planet gear
	planet_gear_sketch->sketchCurves()->sketchCircles()->addByCenterRadius(Point3D::create(planetOrbitRadius + trueEccentricity, 0, 0), planetRadius);
	planet_gear_sketch->sketchCurves()->sketchCircles()->addByCenterRadius(Point3D::create(planetOrbitRadius, 0, 0), planet_shaft_radius);

	ui->messageBox("Finished generating gearbox profile");

	return true;
}

#ifdef XI_WIN

#include <windows.h>

BOOL APIENTRY DllMain(HMODULE hmodule, DWORD reason, LPVOID reserved)
{
	switch (reason)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

#endif // XI_WIN
