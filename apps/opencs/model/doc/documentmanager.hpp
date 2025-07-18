#ifndef CSM_DOC_DOCUMENTMGR_H
#define CSM_DOC_DOCUMENTMGR_H

#include <QObject>
#include <QThread>

#include <filesystem>
#include <string>
#include <vector>

#include <components/files/multidircollection.hpp>
#include <components/toutf8/toutf8.hpp>

#include "loader.hpp"

namespace Files
{
    struct ConfigurationManager;
}

namespace CSMDoc
{
    class Document;

    class DocumentManager : public QObject
    {
        Q_OBJECT

        std::vector<Document*> mDocuments;
        const Files::ConfigurationManager& mConfiguration;
        QThread mLoaderThread;
        Loader mLoader;
        ToUTF8::FromType mEncoding;

        std::filesystem::path mResDir;

        Files::PathContainer mDataPaths;
        std::vector<std::string> mArchives;

        DocumentManager(const DocumentManager&);
        DocumentManager& operator=(const DocumentManager&);

    public:
        DocumentManager(const Files::ConfigurationManager& configuration);

        ~DocumentManager();

        void addDocument(
            const std::vector<std::filesystem::path>& files, const std::filesystem::path& savePath, bool new_);
        ///< \param new_ Do not load the last content file in \a files and instead create in an
        /// appropriate way.

        /// Create a new document. The ownership of the created document is transferred to
        /// the calling function. The DocumentManager does not manage it. Loading has not
        /// taken place at the point when the document is returned.
        ///
        /// \param new_ Do not load the last content file in \a files and instead create in an
        /// appropriate way.
        Document* makeDocument(
            const std::vector<std::filesystem::path>& files, const std::filesystem::path& savePath, bool new_);

        void setResourceDir(const std::filesystem::path& parResDir);

        void setEncoding(ToUTF8::FromType encoding);

        /// Sets the file data that gets passed to newly created documents.
        void setFileData(const Files::PathContainer& dataPaths, const std::vector<std::string>& archives);

        bool isEmpty();

    private slots:

        void documentLoaded(Document* document);
        ///< The ownership of \a document is not transferred.

        void documentNotLoaded(Document* document, const std::string& error);
        ///< Document load has been interrupted either because of a call to abortLoading
        /// or a problem during loading). In the former case error will be an empty string.

    public slots:

        void removeDocument(CSMDoc::Document* document);
        ///< Emits the lastDocumentDeleted signal, if applicable.

        /// Hand over document to *this. The ownership is transferred. The DocumentManager
        /// will initiate the load procedure, if necessary
        void insertDocument(CSMDoc::Document* document);

    signals:

        void documentAdded(CSMDoc::Document* document);

        void documentAboutToBeRemoved(CSMDoc::Document* document);

        void loadRequest(CSMDoc::Document* document);

        void lastDocumentDeleted();

        void loadingStopped(CSMDoc::Document* document, bool completed, const std::string& error);

        void nextStage(CSMDoc::Document* document, const std::string& name, int totalRecords);

        void nextRecord(CSMDoc::Document* document, int records);

        void cancelLoading(CSMDoc::Document* document);

        void loadMessage(CSMDoc::Document* document, const std::string& message);
    };
}

#endif
