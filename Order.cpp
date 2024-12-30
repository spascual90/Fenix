/*
 * Order.cpp
 *
 *  Created on: 20 oct. 2018
 *      Author: Sergio
 */

#include "Order.h"

SERIALorder::SERIALorder() {
};

SERIALorder::~SERIALorder() {
};

e_actions SERIALorder::get_order() const {
	return _order;
};

void SERIALorder::set_order(e_actions order) {
	_order=order;
};

void SERIALorder::reset () {
	APinfo.isValid=NO;
	APinfo.flag = {false, false, false, false, false, false};
	PIDgain.isValid=NO;
	PIDgain.flag = {{false,false,false}, false, false};
	instParam.isValid=NO;
	instParam.flag = {false, false, false, false, false, false};
	APB.isValid=NO;
	APB.flag = {false,false,false,false,false,false,false};
	HDM.isValid=NO;
	HDM.flag = {false};
	VWR.isValid=NO;
	VWR.flag = {false,false};
	calibrate_py.isValid=NO;

	isValid=NO;
	isRequest= false;
	set_order(NO_INSTRUCTION);
}
