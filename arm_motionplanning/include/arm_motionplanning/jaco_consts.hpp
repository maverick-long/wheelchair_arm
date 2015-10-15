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
	GraspObject = 1,
	MovetoHuman = 2,
	SwitchObject = 3,
	ReturntoHomePose = 4
};

}

#endif /* JACO_CONSTS_H_ */