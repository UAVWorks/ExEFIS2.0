#include <QRect>
#include <QPainter>
#include "StatusWidget.h"



StatusWidget::StatusWidget(QWidget *parent)
{
	status = "";
}


StatusWidget::~StatusWidget()
{
	status = "";
}


void StatusWidget::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.setPen(Qt::black);
	
	QRect rec = QRect(0, 0, width(), height());
	painter.drawText(rec, Qt::AlignLeft, "Status: " + status);
}
