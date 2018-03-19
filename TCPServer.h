/** 
 * @file	TCPServer.h 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	February, 2014
 * @brief	Declaration of the TCPServer class.
 */
#ifndef _RDCC_TCP_SERVER
#define _RDCC_TCP_SERVER

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

#include <deque>

class TCPServer;
class TCPMessageSession;
class TCPMessageSession;
class TCPImageSession;

/**
 *@class	TCPMessageSession
 *
 */
class TCPMessageSession{
	public:
		TCPMessageSession(TCPServer* server, boost::asio::io_service* io_service);
		~TCPMessageSession();

		//Setup
		void set_output_message_deque(std::deque<std::string*>* deque);

		//Run
		void start();

		//Messages
		//Write
		void add_message(std::string msg);
		//Read
		int has_new_message();
		bool consume_message(std::string& message);

		//Access
		boost::asio::ip::tcp::socket* get_socket();
		bool is_alive();

		//End
		void stop_session();
	
	private:
		void run_read_thread();
		void run_write_thread();

		//void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
		//void handle_write(const boost::system::error_code& error);

	private:
		TCPServer*						_server;

		bool							_connected;
		bool							_running;

		boost::asio::ip::tcp::socket*	_socket;

		//Read
		static const unsigned int		_buff_size = 1048576;
		char							_read_buffer[_buff_size];
		boost::thread*					_read_thread;

		//Write
		boost::thread*					_write_thread;
		boost::condition_variable		_condition;			/**< Condition variable to wait for new messages.*/
		boost::mutex					_condition_mutex;	/**< Mutex for the contition _condition.*/

		//Queues
		boost::mutex					_write_mutex;		/**< Mutex for the write queue.*/
		std::deque<std::string*>*		_write_deque;		/**< DeQueue of messages to be send.*/
		boost::mutex					_read_mutex;		/**< Mutex for the read queue.*/
		std::deque<std::string*>*		_read_deque;		/**< DeQueue of messages read.*/
};

/**
 *@class	TCPImageSession
 *
 */
class TCPImageSession{
	public:
		TCPImageSession(TCPServer* server, boost::asio::io_service* io_service);
		~TCPImageSession();

		//Run
		void start();

		//Message
		void add_image(cv::Mat image);

		//Access
		boost::asio::ip::tcp::socket* get_socket();
		bool is_alive();

		//End
		void stop_session();

	private:
		void run_read_thread();
		void run_write_thread();

	private:
		TCPServer*						_server;

		bool							_connected;
		bool							_running;

		boost::asio::ip::tcp::socket*	_socket;

		//Read
		static const unsigned int		_buff_size = 1048576;
		char							_read_buffer[_buff_size];
		boost::thread*					_read_thread;

		//Write
		boost::thread*					_write_thread;
		boost::condition_variable		_condition;			/**< Condition variable to wait for new messages.*/
		boost::mutex					_condition_mutex;	/**< Mutex for the contition _condition.*/

		//Queues
		boost::mutex					_write_mutex;		/**< Mutex for the write queue.*/
		std::deque<cv::Mat>*			_write_deque;		/**< DeQueue of messages to be send.*/

};

/**
 *@class	TCPImageSession
 *
 */
class TCPServer{
	public:
		enum FLAGS{
			COMMUNICATION_ON,
			MESSAGE_ALL
		};
		static const int _n_flags = 2;

	public:
		TCPServer(int app_port = 7777, int setup_port = 13000, int image_port = 13001);
		~TCPServer();

		//Set info
		void set_ip(char* ip);		
		void set_app_port(int port);
		void set_setup_port(int port);
		void set_image_port(int port);
		

		void init_server();
		void stop_server();

		//Get info
		std::string* get_ip();
		int get_app_port();
		int get_setup_port();
		int get_image_port();

		//Check for new messages
		bool has_new_message();
		bool has_new_app_message();
		bool has_new_setup_message();

		//Consume messages
		void consume_app_message(std::string& message);
		void consume_setup_message(std::string& message);

		//Add message to be send 
		void send_app_message(std::string msg);
		void send_setup_message(std::string msg);
		void send_image_message(cv::Mat image);

		//XML
		bool save_calibration(FILE *file);
		bool load_calibration(FILE *file);

	private:
		
		void run_tcp_thread();
		void run_maintenance_thread();

		void handle_accept_app(TCPMessageSession* new_session, const boost::system::error_code& error);
		void handle_accept_setup(TCPMessageSession* new_session, const boost::system::error_code& error);
		void handle_accept_image(TCPImageSession* new_session, const boost::system::error_code& error);

		void manage_messages();
		void manage_app_session();
		void manage_setup_session();
		void manage_image_session();

	private:
		void add_app_message_to_queue(std::string msg);
		void add_setup_message_to_queue(std::string msg);

	
	public:
		bool										_flags[_n_flags];

	private:
		std::string									_host;

		short										_app_port_short;
		short										_setup_port_short;
		short										_image_port_short;

		boost::asio::io_service*					_io_service;	/**< Connection's IO Service.*/
		boost::thread*								_io_thread;

		bool										_running;
		boost::thread*								_maintenance_thread;
		boost::mutex								_mutex;			/**< Mutex for the contition _condition.*/
		boost::condition_variable					_condition;		/**< Condition variable to wait for new messages.*/


		boost::asio::ip::tcp::acceptor*				_app_acceptor;
		TCPMessageSession*							_app_session;
		std::deque<std::string*>					_app_messages_in;		/**< DeQueue of messages received.*/
		std::deque<std::string*>					_app_messages_out;		/**< DeQueue of messages to be send.*/	
		
		boost::asio::ip::tcp::acceptor*				_setup_acceptor;
		TCPMessageSession*							_setup_session;
		std::deque<std::string*>					_setup_messages_in;		/**< DeQueue of messages received.*/
		std::deque<std::string*>					_setup_messages_out;	/**< DeQueue of messages to be send.*/	

		boost::asio::ip::tcp::acceptor*				_image_acceptor;
		TCPImageSession*							_image_session;


};

#endif//_RDCC_TCP_SERVER