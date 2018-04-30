#include "mainwindow.h"

#include <QApplication>


int main(int argc, char *argv[])
{
	printf("main\n");
	QApplication *a = new QApplication(argc, argv);
	MainWindow *w = new MainWindow();
	w->show();

	return a->exec();
}
