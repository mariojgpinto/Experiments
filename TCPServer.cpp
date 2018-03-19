/** 
 * @file	TCPServer.cpp 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	February, 2014
 * @brief	Implementation of the TCPServer class.
 */
#include "TCPServer.h"

#include <iostream>

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
TCPServer::TCPServer(int app_port, int setup_port, int image_port){
	this->_app_port_short   = app_port;
	this->_setup_port_short = setup_port;
	this->_image_port_short = image_port;

	this->_app_session = NULL;
	this->_setup_session = NULL;
	this->_image_session = NULL;

	this->_host = "localhost";

	for(int i = 0 ; i < TCPServer::_n_flags; ++i){
		this->_flags[i] = false;
	}
	//this->init_server();
}

/**
 * @brief	.
 * @details	.
 */
TCPServer::~TCPServer(){

}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPServer::init_server(){
	this->_io_service = new boost::asio::io_service();

	this->_app_acceptor = new boost::asio::ip::tcp::acceptor(*_io_service, 
								boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), this->_app_port_short));
	TCPMessageSession* next_session = new TCPMessageSession(this, this->_io_service);
	this->_app_acceptor->async_accept(	*next_session->get_socket(),
										boost::bind(&TCPServer::handle_accept_app, 
													this, next_session,
													boost::asio::placeholders::error));

	this->_setup_acceptor = new boost::asio::ip::tcp::acceptor(*_io_service, 
								boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), this->_setup_port_short));

	TCPMessageSession* next_session_setup = new TCPMessageSession(this, this->_io_service);
	this->_setup_acceptor->async_accept(*next_session_setup->get_socket(),
										boost::bind(&TCPServer::handle_accept_setup, 
										this, next_session_setup,
										boost::asio::placeholders::error));

	this->_image_acceptor = new boost::asio::ip::tcp::acceptor(*_io_service, 
								boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), this->_image_port_short));

	TCPImageSession* next_session_image = new TCPImageSession(this, this->_io_service);
	this->_image_acceptor->async_accept(*next_session_image->get_socket(),
										boost::bind(&TCPServer::handle_accept_image, 
										this, next_session_image,
										boost::asio::placeholders::error));

	
	std::cout << "Socket Server Initialized on Port " << this->_app_port_short << ".\n";

	this->_io_thread = new boost::thread(&TCPServer::run_tcp_thread,this);
	this->_maintenance_thread = new boost::thread(&TCPServer::run_maintenance_thread,this);
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::stop_server(){
	this->_running = false;
	this->_condition.notify_all();

	if(this->_app_session){
		//this->_app_session->get_socket()->close();
		this->_app_session->stop_session();
	}

	if(this->_setup_session){
		//this->_setup_session->get_socket()->close();
		this->_setup_session->stop_session();
	}

	if(this->_image_session){
		//this->_image_session->get_socket()->close();
		this->_image_session->stop_session();
	}
	try{
		this->_io_service->stop();
	}
	catch(...){
		printf("Error on\"TCPServer\" on \"_io_service->stop()\"\n");
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::handle_accept_app(TCPMessageSession* new_session, const boost::system::error_code& error){
	if (!error){
		printf("New App Connection\n");
		//Initialize new Session
		this->_app_session = new_session;
		this->_app_session->set_output_message_deque(&this->_app_messages_in);
		this->_app_session->start();

		//Wake Up Maintenance Thread for missing messages
		this->_condition.notify_all();
    }
    else{
		delete new_session;
    }
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::handle_accept_setup(TCPMessageSession* new_session, const boost::system::error_code& error){
	if (!error){
		printf("New Setup Connection\n");
		//Initialize new Session
		new_session->start();

		Sleep(100);

		new_session->set_output_message_deque(&this->_setup_messages_in);
		this->_setup_session = new_session;
		

		//Wake Up Maintenance Thread for missing messages
		this->_condition.notify_all();
    }
    else{
		delete new_session;
    }
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::handle_accept_image(TCPImageSession* new_session, const boost::system::error_code& error){
	if (!error){
		printf("New Image Connection\n");
		//Initialize new Session
		new_session->start();

		Sleep(100);

		this->_image_session = new_session;
		

		//Wake Up Maintenance Thread for missing messages
		this->_condition.notify_all();
    }
    else{
		delete new_session;
    }
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::set_ip(char* ip){
	if(ip)
		this->_host.assign(ip);
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::set_app_port(int port){
	this->_app_port_short = (short)port;
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::set_setup_port(int port){
	this->_setup_port_short = (short)port;
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::set_image_port(int port){
	this->_image_port_short = (short)port;
}

//-----------------------------------------------------------------------------
// RUN
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPServer::run_tcp_thread(){
	this->_io_service->run();
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::run_maintenance_thread(){
	this->_running = true;

	while(this->_running){
		this->manage_app_session();
		this->manage_setup_session();
		this->manage_image_session();

		this->manage_messages();

		{
			boost::mutex::scoped_lock lock(this->_mutex);
			this->_condition.timed_wait(lock,boost::posix_time::millisec(1000));
		}
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::manage_app_session(){
	if(this->_app_session != NULL && !this->_app_session->is_alive()){
		//Destroy current session
		this->_app_session->~TCPMessageSession();

		//Listen for another session
		TCPMessageSession* next_session = new TCPMessageSession(this, _io_service);
		this->_app_acceptor->async_accept(	*next_session->get_socket(),
											boost::bind(&TCPServer::handle_accept_app, this, next_session,
														boost::asio::placeholders::error));
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::manage_setup_session(){
	if(this->_setup_session && !this->_setup_session->is_alive()){
		//Destroy current session
		this->_setup_session->~TCPMessageSession();
		this->_setup_session = NULL;

		//Listen for another session
		TCPMessageSession* next_session = new TCPMessageSession(this, _io_service);
		this->_setup_acceptor->async_accept(	*next_session->get_socket(),
												boost::bind(&TCPServer::handle_accept_setup, this, next_session,
															boost::asio::placeholders::error));
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::manage_image_session(){
	if(this->_image_session && !this->_image_session->is_alive()){
		//Destroy current session
		this->_image_session->~TCPImageSession();
		this->_image_session = NULL;

		//Listen for another session
		TCPImageSession* next_session = new TCPImageSession(this, _io_service);
		this->_image_acceptor->async_accept(	*next_session->get_socket(),
												boost::bind(&TCPServer::handle_accept_image, this, next_session,
															boost::asio::placeholders::error));
	}
}

//-----------------------------------------------------------------------------
// MESSAGES
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPServer::manage_messages(){
	if(this->_app_messages_out.size() && this->_app_session){
		while(this->_app_messages_out.size()){
			if(this->_app_messages_out[0]){
				this->send_app_message(*this->_app_messages_out[0]);
			}
			this->_app_messages_out.pop_front();
		}
	}

	if(this->_setup_messages_out.size() && this->_setup_session){
		while(this->_setup_messages_out.size()){
			if(this->_setup_messages_out[0]){
				this->send_setup_message(*this->_setup_messages_out[0]);
			}
			this->_setup_messages_out.pop_front();
		}
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::send_app_message(std::string msg){
	if(this->_app_session && this->_app_session->is_alive()){
		this->_app_session->add_message(msg);
	}
	else{
		this->add_app_message_to_queue(msg);
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::add_app_message_to_queue(std::string msg){
	this->_app_messages_out.push_back(new std::string(msg));
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::send_setup_message(std::string msg){
	if(this->_setup_session && this->_setup_session->is_alive()){
		this->_setup_session->add_message(msg);
	}
	else{
		this->add_setup_message_to_queue(msg);
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::add_setup_message_to_queue(std::string msg){
	this->_setup_messages_out.push_back(new std::string(msg));
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::send_image_message(cv::Mat image){
	if(this->_image_session && this->_image_session->is_alive()){
		this->_image_session->add_image(image);
	}
	else{
		//this->send_app_message_to_queue(msg);
		//discard
	}
}

//-----------------------------------------------------------------------------
// MESSAGE ACCESS
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
bool TCPServer::has_new_message(){
	return (this->_app_messages_in.size() || this->_setup_messages_in.size()) ? true : false;
}

/**
 * @brief	.
 * @details	.
 */
bool TCPServer::has_new_app_message(){
	return (this->_app_messages_in.size()) ? true : false;
}

/**
 * @brief	.
 * @details	.
 */
bool TCPServer::has_new_setup_message(){
	return (this->_setup_messages_in.size()) ? true : false;
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::consume_app_message(std::string& message){
	if(this->_app_messages_in.size()){
		if(this->_app_messages_in[0]){
			message.assign(this->_app_messages_in[0]->data());
		}
		this->_app_messages_in.pop_front();
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::consume_setup_message(std::string& message){
	if(this->_setup_messages_in.size()){
		if(this->_setup_messages_in[0]){
			message.assign(this->_setup_messages_in[0]->data());
		}
		this->_setup_messages_in.pop_front();
	}
}

//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------

/**
 * @brief	.
 * @details	.
 */
std::string* TCPServer::get_ip(){
	return &this->_host;
}

/**
 * @brief	.
 * @details	.
 */
int TCPServer::get_app_port(){
	return this->_app_port_short;
}

/**
 * @brief	.
 * @details	.
 */
int TCPServer::get_setup_port(){
	return this->_setup_port_short;
}

/**
 * @brief	.
 * @details	.
 */
int TCPServer::get_image_port(){
	return this->_image_port_short;
}

//-----------------------------------------------------------------------------
// XML
//-----------------------------------------------------------------------------

/**
 * @brief	.
 * @details	.
 */
bool TCPServer::save_calibration(FILE *file){
	if(!file)
		return false;
	
	fprintf(file,"Communication ip=%s app_port=%d setup_port=%d image_port=%d\n",this->_host.data(), 
																				(int)this->_app_port_short,
																				(int)this->_setup_port_short,
																				(int)this->_image_port_short);

	return true;
}

/**
 * @brief	.
 * @details	.
 */
bool TCPServer::load_calibration(FILE *file){
	if(!file)
		return false;
	
	char buff[1024];

	char host[128];
	int app_port, setup_port, image_port;
	fgets(buff,1024,file);
	sscanf(buff,"Communication ip=%s app_port=%d setup_port=%d image_port=%d\n",host,&app_port,&setup_port,&image_port);

	this->set_app_port(app_port);
	this->set_setup_port(setup_port);
	this->set_image_port(image_port);

	this->set_ip(host);

	return true;
}