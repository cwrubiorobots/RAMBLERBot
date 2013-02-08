/**
*************************************************************
* @file LList.h
* Another implementation of linked lists to use in Arduino.
*    Copyright (C) 2011 Gaspar Fernández
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
* @brief Linked list implementation for Arduino
*
* Features:
*   - Can access to head and tail directly
*   - Can pop back and front from this class
*   - Can access any item from the item number
*   - Can set a callback function when an error occurs
*
* @author Gaspar Fernández <blakeyed@totaki.com>
* @web http://totaki.com/poesiabinaria
* @version 0.1
* @date 27 nov 2011
* Change history
*
* To-do:
*   - Better control over non-critical errors, maybe another callback
*   - Pre-defined error methods: by serial, by output leds...
*************************************************************/

#ifndef LList_h
#define LList_h

#include <Arduino.h>

#define LLIST_ERROR_NOMEMORY     1 // Not enough memory to insert element
#define LLIST_ERROR_NOTAIL       2 // Can't get last items
#define LLIST_ERROR_NOFRONT      3 // Can't get head items
#define LLIST_ERROR_NOITEMS      4 // List empty, can't read items
#define LLIST_ERROR_NEITEMS      5 // Not enough items (pos>=sz)

template <typename Type>
class LList
{
public:
  typedef void (*ErrorCallback)(int);

  // We can add more error types 
  enum ListErrorType
    {
      NO_ERROR_REPORTING,	// No error reporting
      CALLBACK			// Errors will trigger a callback
    };

  // Destruction. Clear data
  ~LList();

  // Initialization
  LList();

  // Inserts new item in the last position
  void push_back(const Type item);
  // Is the list empty?
  inline bool isEmpty() const;
  // Returns the size of the list
  inline int size();
  // Returns the last item
  inline Type back();
  // Returns the first item
  inline Type front();
  // Return the last item and delete it
  Type pop_back();
  // Return the first item and delete it
  Type pop_front();

  // Get the item at position pos
  Type getElement(unsigned pos);
  // Insert a new item at position pos
  void insert(int pos, Type item);
  // set error callback function
  void setErrorCallback(ErrorCallback cback);
  // no error reporting
  void setNoErrorReporting();
  // clears the list
  void clear();
private:
  // Reports an error
  void report_error(int err);
  
  // Deletes a list when there is only one element
  void delete_list();

  typedef struct node 
  {
    Type item;
    node *next;
  } node;

  node *head;			// Pointer to the head
  node *tail;			// Pointer to the tail
  int sz;			// List size
  ListErrorType errorType;	
  ErrorCallback ecback;		// Callback when reporting an error
};

template <typename Type>
LList<Type>::LList()
{
  head=NULL;
  tail=NULL;
  sz=0;
  errorType=NO_ERROR_REPORTING;
}

template <typename Type>
LList<Type>::~LList()
{
  clear();			// Clears the list before destroying
}

template <typename Type>
void LList<Type>::push_back(const Type item)
{
  node *aux=tail;
  
  // Reserve memory for a new node
  tail=(node*)malloc(sizeof(node));
  if (tail==NULL)
    {
      report_error(LLIST_ERROR_NOMEMORY);
      return;
    }

  // Stores the item information
  tail->item=item;
  tail->next=NULL;

  // Link to the list
  if (isEmpty())
    head=tail;
  else
    aux->next=tail;

  sz++;

}

template <typename Type>
bool LList<Type>::isEmpty() const
{
  return (sz==0);		// (sz==0) = EMPTY
}

template <typename Type>
int LList<Type>::size()
{
  return sz;
}

template <typename Type>
inline Type LList<Type>::back()
{
  if (isEmpty())		// List empty = No last item
    {
      Type t;
      report_error(LLIST_ERROR_NOTAIL);
      return t;
    }

  return tail->item;
}

template <typename Type>
inline Type LList<Type>::front()
{
  if (isEmpty())		// List empty = No first item
    {
      Type t;
      report_error(LLIST_ERROR_NOFRONT);
      return t;
    }

  return head->item;
}

template <typename Type>
Type LList<Type>::pop_back()
{ 
  node *aux=head;
  Type t;

  if (isEmpty())		// List empty = No last item
    {
      report_error(LLIST_ERROR_NOTAIL);
      return t;
    }
  
  if (head==tail)		// If there is only 1 item
    {
      t=head->item;
      delete_list();
      return t;
    }
  else
    {				// More than one item
      while (aux->next!=tail)	// Searches for the item previous to the last
	aux=aux->next;
      
      t=tail->item;		// I will return the tail item
      free(tail);		
      tail=aux;			// Define the new tail
      tail->next=NULL;
      sz--;			// 1 item less

      return t;
    }
}

template <typename Type>
Type LList<Type>::pop_front()
{
  Type t;
  node *aux=head;
  
  if (isEmpty())		// List empty = No head
    {
      report_error(LLIST_ERROR_NOFRONT);
      return t;
    }

  t=aux->item;			// I will return the head
  head=head->next;		// The new head is the item after the head
  free(aux);
  if (head==NULL)		// If I had deleted the last item, replace the tail
    tail=NULL;			// too.
  sz--;

  return t;
}

template <typename Type>
void LList<Type>::delete_list()
{
  free(head);			// I will call this method when deleting
  head=NULL;			// a list with only one element, so all
  tail=NULL;			// the variables are set.
  sz=0;
}

template <typename Type>
void LList<Type>::insert(int pos, Type item)
{
  node *newitem;
  node *aux=head;
  int i;

  // Allocate memory for the new item
  newitem=(node*)malloc(sizeof(node)); 
  if (newitem==NULL)
    {
      report_error(LLIST_ERROR_NOMEMORY);
      return;
    }
  // Stores the item information
  newitem->item=item;
  newitem->next=NULL;

  // Link the item to the list
  if (isEmpty())
    {				// If the list is empty there will be only
      head=newitem;		// one item, so it will be the head and the
      tail=newitem;		// tail.
    }
  else if (pos==0)		// If the item is going to be inserted at the beginning
    {
      newitem->next=head;	// we will move the head
      head=newitem;
    }
  else if (pos>=sz)		// If the position is greater than the last item's position
    {				// it will be inserted after this item, at the tail.
      tail->next=newitem;      
      tail=newitem;
    }
  else
    {				// If not, we will have to find the position of the 
      i=0;			// previous item to link its next pointer to the new
      while (i<pos-1)		// element.
	{
	  aux=aux->next;
	  ++i;
	}
      newitem->next=aux->next;	// And then link the next pointer of our new element
      aux->next=newitem;	// to where the next pointer of the previour element
    }				// was pointing.
  sz++;
}

template <typename Type>
Type LList<Type>::getElement(unsigned pos)
{
  int i=0;
  Type t;
  node *aux=head;

  if (isEmpty())		// If the list is empty = No data
    {
      report_error(LLIST_ERROR_NOITEMS);
      return t;
    }

  if (pos>=sz)			// If the item asked for is greater than the
    {				// last item's position. There is no valid element
      report_error(LLIST_ERROR_NEITEMS);
      return t;
    }
  
  if (pos==sz-1)		// If the item we asked for is the last, we can 
    return tail->item;		// return it inmediately


  while (i<pos)			// If not, we must look for it
    {
      aux=aux->next;
      i++;
    }
  return aux->item;
}

template <typename Type>
void LList<Type>::setErrorCallback(ErrorCallback cback)
{
  ecback=cback;			// This method sets the error callback to invoke
  errorType=CALLBACK;		// when an error occurs.
}

template <typename Type>
void LList<Type>::report_error(int err)
{
  if (errorType==CALLBACK)	// If the error reporting is through a callback,
    {				// call it.
      ecback(err);
    }
}

template <typename Type>
void LList<Type>::clear()
{
  node *aux;

  while (head!=NULL)		// Delete all elements
    {
      aux=head;
      head=head->next;
      free(aux);
    }
  
  tail=NULL;			// Restore variable stats
  sz=0;
}  

template <typename Type>
void LList<Type>::setNoErrorReporting()
{
  errorType=NO_ERROR_REPORTING;
}

#endif

// Test program
// LList<int> l;

// void blink(int err)
// {
//   while (1)
//     {
//       digitalWrite(11,HIGH);
//       delay(1000);
//       digitalWrite(11, LOW);
//       delay(1000);   
//     }
// }

// void setup()
// {
//   pinMode(11, OUTPUT);
//   Serial.begin(9600);
//   l.setErrorCallback(blink);
//   l.push_back(45);
//   l.push_back(42);
//   l.push_back(47);
//   l.push_back(89);
//   l.insert(0, 33);
//   l.insert(1, 53);
//   l.insert(9, 73);
//   // 33
//   // 53
//   // 45
//   // 42
//   // 47
//   // 89
//   // 73
// }

// void loop()
// {
//   char r;
//   if (Serial.available()>0)
//     {
//       while (Serial.available()>0)
// 	{
// 	  r=Serial.read();
// 	  delayMicroseconds(10000);
// 	}

//       for (unsigned i=0; i<l.size(); i++)
// 	{
// 	  Serial.println(l.getElement(i), DEC);
// 	}
//       Serial.print("Elements: ");
//       Serial.println(l.size(), DEC);
//       Serial.print("Last: ");
//       Serial.println(l.pop_front(), DEC);
//     }
// }