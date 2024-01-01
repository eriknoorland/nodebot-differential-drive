typedef void (*RequestHandlerFunc)();

struct RequestHandler {
  byte command;
  RequestHandlerFunc function;
};