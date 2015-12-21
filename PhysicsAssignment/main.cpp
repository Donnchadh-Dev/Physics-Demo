// A project by Jan and Donnchadh

#include "BulletOpenGLApplication.h"
#include "FreeGLUTCallbacks.h"

int main(int argc, char** argv)
{
	BulletOpenGLApplication demo;
	return glutmain(argc, argv, 1024, 768, "Domino Simulation Using Bullet Physics Engine", &demo);
}