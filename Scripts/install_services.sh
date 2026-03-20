chmod +x ../services/*
cp ../services/* /etc/supervisor/conf.d/
supervisorctl reread
supervisorctl update