#include "actserver.h"

class MyServer: public ActServer {
public:
	// errors
	enum Err {
		ERR_OK,						// SIMU, INIT, VERB, DATA
		ERR_INIT_NEEDED,			// DATA
	};
};

//
//  MyServerMsgDATA - DATA: My Server data request
//
//  Same message structure used to set the target gear and to get the
//  current gear state.
//  To set the target gear, use get=false.
//  To get the current gear state, use get=true.
//
struct MyServerMsgDATA: public MsgBase {
	static const uint32_t k_msgtype = char4('D','A','T','A');
	MyServer::Err err;				// returned, ERR_0K=no error,otherwise error
	bool get;						// true=get gear state, false=set state
	int data;						// data for illustrative purposes
};

//
//  MyServerMsg - all My Server messages as a union
//
//  Used as argument to MsgReceive. Size of union is size of largest 
//  acceptable message.
//
union MyServerMsg {
	ActServerMsgSIMU m_simu;
	ActServerMsgINIT m_init;
	ActServerMsgVERB m_verb;
	MyServerMsgDATA m_data;
};