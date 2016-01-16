/*Created by Aaron Mishkin - January 14, 2016 */

/**
 * Interface for maps
 */

#ifndef __MAP_INTERFACE__
#define __MAP_INTERFACE__

class map_interface{
	public:
	virtual int at(int x, int y) const = 0;
	virtual int width() const = 0;
	virtual int height() const = 0;
	virtual void set(int x, int y, int value) = 0;
	virtual bool withinBounds(int x, int y) = 0;
	virtual ~map_interface(){}
};

#endif