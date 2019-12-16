import os, sys

def print_exception(e):
  exc_type, exc_obj, exc_tb = sys.exc_info()
  fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
  print(exc_type,',', fname,', ln', exc_tb.tb_lineno)
  print(e)
  print("Closing socket")