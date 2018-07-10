#include "SplashWidget.h"
#include "panelWidget.h"
#include "DiagWidget.h"
#include "knobs.h"
#include "adhrs.h"
#include <QApplication>
#include <QStackedWidget>
#include <QProcess>
#include <QWidget>
#include <QScreen>

int main(int argc, char *argv[])
{	
	QStringList list = QProcessEnvironment::systemEnvironment().toStringList();
		

	
	system("gpio edge 12 both");
	system("gpio edge 13 both");
	system("gpio edge 19 both");
	
	system("gpio edge 5 both");
	system("gpio edge 6 both");
	system("gpio edge 16 both");
	
	//system("gpio load spi");
	
	//bcm2835_init();
	
	qDebug("****************** LOG BEGINS HERE **************************");
	qDebug("Version 1.1 - Kitfox Test");
	
	QApplication a(argc, argv);
	
	QApplication::setOverrideCursor(Qt::BlankCursor);

	QScreen *screen = QGuiApplication::primaryScreen();
	QRect  screenGeometry = screen->geometry();
	
	knobs *k = new knobs();
	adhrs *ad = new adhrs();
	
	
	QStackedWidget *w = new QStackedWidget();
	
	panelWidget *p = new panelWidget();
	p->setADHRS(ad);
	p->setKNOBS(k);
	p->showFullScreen();
	p->update();
	
	p->setCursor(Qt::BlankCursor);
	
	
//	SplashWidget *s = new SplashWidget(0, ad, k);
//	s->showFullScreen();
//	s->update();
	
	//DiagWidget *m = new DiagWidget();
	//m->setADHRS(ad);
	//m->setKnobs(k);
	//m->showFullScreen();
	
//	w->addWidget(s);
//	w->addWidget(p);
//	w->addWidget(m);
	
	//s->setParent(w);
	//p->setParent(p);
	//m->setParent(m);
	//QStackedWidget::connect(p, SIGNAL(launchDiag(int)), w, SLOT(setCurrentIndex(int)));
//	QStackedWidget::connect(s, SIGNAL(launchPanel(int)), w, SLOT(setCurrentIndex(int)));
//	QApplication::connect(s, SIGNAL(closeStacked()), w, SLOT(quit()));
	w->setCurrentIndex(0);
//	w->showFullScreen();
//	w->update();
	
	//panelWidget *w = new panelWidget();
	//w->showFullScreen();
	

	
    return a.exec();
}
