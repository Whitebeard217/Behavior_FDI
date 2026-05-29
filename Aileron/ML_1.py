import pandas as pd
from matplotlib import pyplot
from sklearn.model_selection import train_test_split
from sklearn.model_selection import KFold
from sklearn.model_selection import cross_val_score
from sklearn.metrics import classification_report
from sklearn.metrics import confusion_matrix
from sklearn.metrics import accuracy_score
from pickle import dump
from pickle import load
from sklearn.metrics import recall_score
from sklearn.metrics import precision_score
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.neighbors import KNeighborsClassifier
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import MinMaxScaler
from sklearn.preprocessing import Normalizer
names = ['Force', 'Displacement', 'Current', 'Angle_Meas', 'Angle_Cmd', 'State', 'Fault', 'Event'] 
#dataset = pd.read_excel(r"G:\Other computers\My laptop\Aircraft_project\Event_dataset.xlsx", names=names)
dataset = pd.read_excel(r"C:\Aircraft_project\Event_dataset.xlsx", names=names)

array = dataset.values
X = array[:,0:4]
Y = array[:,7]
scaler = StandardScaler().fit(X)
rescaledX = scaler.transform(X)
scaler = MinMaxScaler(feature_range=(0, 1))
rescaledX1 = scaler.fit_transform(rescaledX)
scaler = Normalizer().fit(rescaledX1)
X = scaler.transform(rescaledX1)
test_size = 0.33
seed = 7
validation_size = 0.20
#seed = 7
X_train, X_validation, Y_train, Y_validation = train_test_split(X, Y, test_size=validation_size, random_state=seed)
#X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=test_size, random_state=seed)
num_folds = 10
models = []
models.append(('LR', LogisticRegression(max_iter=1000)))
models.append(('LDA', LinearDiscriminantAnalysis()))
models.append(('KNN', KNeighborsClassifier()))
models.append(('CART', DecisionTreeClassifier()))
models.append(('NB', GaussianNB()))
models.append(('SVM', SVC()))

# evaluate each model in turn
results = []
names = []
scoring = 'accuracy'
for name, model in models:
    kfold = KFold(n_splits=10, random_state=7, shuffle=True)
    cv_results = cross_val_score(model, X_train, Y_train, cv=kfold, scoring=scoring)
    results.append(cv_results)
    names.append(name)
    msg = "%s: %f (%f)" % (name, cv_results.mean(), cv_results.std())
    print(msg)
# boxplot algorithm comparison
fig = pyplot.figure()
fig.suptitle('Algorithm Comparison')
ax = fig.add_subplot(111)
pyplot.boxplot(results)
ax.set_xticklabels(names)
pyplot.show()
knn = KNeighborsClassifier()
knn.fit(X_train, Y_train)
predictions = knn.predict(X_validation)
print(accuracy_score(Y_validation, predictions))
print(confusion_matrix(Y_validation, predictions))
print(classification_report(Y_validation, predictions))
filename = 'finalized_model.sav'
dump(knn, open(filename, 'wb'))
'''Precision:
Precision is the ratio of correctly predicted positive instances to the total predicted positive instances.
It answers the question: "Of all the instances the model labeled as positive, how many actually were positive?"
Formula: Precision = True Positives / (True Positives + False Positives)
A high precision indicates a low false positive rate.
Recall (also known as Sensitivity or True Positive Rate):
Recall is the ratio of correctly predicted positive instances to all actual positive instances.
It answers the question: "Of all the actual positive instances, how many did the model correctly identify?"
Formula: Recall = True Positives / (True Positives + False Negatives)
A high recall indicates a low false negative rate.
F1-score:
The F1-score is the harmonic mean of precision and recall, providing a single score that balances both metrics.
It is particularly useful when you have an uneven class distribution.
Formula: F1 = 2 * (Precision * Recall) / (Precision + Recall)
A high F1-score indicates that both precision and recall are high.
Support:
Support is the number of actual occurrences of the class in the specified dataset.
It provides context for the other metrics by showing the class distribution.
This is important because precision, recall, and F1-score can be misleading for imbalanced datasets'''