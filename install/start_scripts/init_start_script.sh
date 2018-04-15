#!/bin/sh


cp start_flyMS.service /lib/systemd/system/

systemctl daemon-reload
systemctl enable start_flyMS


