#pragma once
#include <QWidget>

class StatusWidget:public QWidget
{
public:
	StatusWidget(QWidget *parent);
	~StatusWidget();
	QString status;
	
protected: 
	void paintEvent(QPaintEvent *event) override;
};

