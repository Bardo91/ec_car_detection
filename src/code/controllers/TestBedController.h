///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	EUROARMS:	Bar Task
//		Author:	Pablo Ramon Soria
//		Date:	2015-FEB-26
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//

#ifndef TEST_BED_CONTROLLER_H_
#define TEST_BED_CONTROLLER_H_


#include "Controller.h"

class TestBedController : public Controller{
private:
	void commandAction(const Message& _message){
		switch (_message.type())
		{
		case MessageType::eLanding:
			std::cout << "Controller: Landing drone" << std::endl;
			break;
		case MessageType::eTakeOff:
			std::cout << "Controller: Taking off drone" << std::endl;
			break;
		case MessageType::ePosition:
			std::cout << "Controller: Drone moving to: " << _message.payload() << std::endl;
			break;
		default:
			std::cout << "Controller: Message of type: " << _message.type() << " is not supported in this controller." << std::endl;
			break;
		}
	}
};

#endif	//	TEST_BED_CONTROLLER_H_