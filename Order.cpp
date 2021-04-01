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

	//TODO: TO CHECK ALTERNATIVE CODING
//	APinfo = {};
//	PIDgain = {};
//	instParam = {};
//	APB = {};
	APinfo.isValid=NO;
	APinfo.flag = {false, false, false, false, false, false};
	PIDgain.isValid=NO;
	PIDgain.flag = {{false,false,false}, false, false};
	instParam.isValid=NO;
	instParam.flag = {false, false, false, false, false, false}; //TODO: Check this warning
	APB.isValid=NO;
	APB.flag = {false,false,false,false,false,false,false};
	isValid=NO;
	isRequest= false;
	set_order(NO_INSTRUCTION);
}
