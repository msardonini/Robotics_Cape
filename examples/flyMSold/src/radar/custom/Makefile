all: radar/mavlink.h radar.py
radar/mavlink.h : radar.xml common.xml
	export PYTHONLIB= ;\
	mavgen.py -o . --lang C --wire-protocol 2.0 radar.xml
radar.py : radar.xml common.xml
	export PYTHONLIB= ;\
	mavgen.py -o radar.py --lang Python --wire-protocol 2.0 radar.xml
common.xml : ../v2.0/message_definitions/common.xml
	if [ -f common.xml ]; then \
		rm common.xml; \
	fi
	ln -s ../v2.0/message_definitions/common.xml .
