all:
	rm -rf ahrs*.so build/ ahrs.cpp  __pycache__
	python3 setup.py build_ext --inplace 
	./test_ahrs.py

clean:
	rm -rf ahrs*.so build/ ahrs.cpp  __pycache__

