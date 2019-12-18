import os, sys

def print_exception(e):
  exc_type, exc_obj, exc_tb = sys.exc_info()
  fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
  template = "An exception of type {0} occurred. Arguments:\n{1!r}"
  message = template.format(type(e).__name__, e.args)
  print()
  print("/////////////////////////////////////////////////>>>>>>>>>>>>>>>>>>>>>>>>>>>")
  print(exc_type,',', fname,', ln', exc_tb.tb_lineno)
  print(message)
  print(e)