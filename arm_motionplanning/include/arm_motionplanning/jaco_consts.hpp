/*
 * jaco_consts.h
 *
 *  Created on: Aug 31, 2015
 *      Author: Xianchao
 */

#ifndef JACO_CONSTS_H_
#define JACO_CONSTS_H_

namespace jaco_traj{

enum class TrajoptMode : int {
	Custom = -1,
	Default = 0,
	Door = 1,
	Debris = 2,
	Drill = 3,
	Valve = 4,
	Drag = 5,
	Pull = 6,
	DebrisRemove = 7,
	ValveRotate = 8,
	ValveRemove = 9,
	DrillGrasp = 10,
	DrillRotate = 11,
	DrillGrasp2 = 12
};

}

#endif /* JACO_CONSTS_H_ */